#!/usr/bin/env python3
"""FPV drone analog signal detector + real-time preview for HackRF/SoapySDR.

Features:
- Wideband scanning (0..6 GHz configurable)
- 5.8 GHz FPV preset scan
- Candidate ranking using RF power + simple video-likeness heuristic
- Real-time watch mode with rough FM analog video reconstruction

This is a practical prototype intended as a starting point.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_CF32, SOAPY_SDR_RX


FPV_58_PRESET_MHZ = [
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,  # Raceband
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,  # FatShark band (legacy)
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,  # Boscam E
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,  # Boscam A
]


@dataclass
class ScanResult:
    freq_hz: float
    power_db: float
    video_score: float

    @property
    def combined(self) -> float:
        return self.power_db + self.video_score


class HackRFReceiver:
    def __init__(
        self,
        sample_rate: float,
        bandwidth: Optional[float],
        lna_gain: float,
        vga_gain: float,
        amp: bool,
    ) -> None:
        self.sample_rate = sample_rate
        self.bandwidth = bandwidth
        self.lna_gain = lna_gain
        self.vga_gain = vga_gain
        self.amp = amp

        self.sdr = SoapySDR.Device(dict(driver="hackrf"))
        self.sdr.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)
        if bandwidth:
            self.sdr.setBandwidth(SOAPY_SDR_RX, 0, bandwidth)

        self.sdr.setGain(SOAPY_SDR_RX, 0, "LNA", lna_gain)
        self.sdr.setGain(SOAPY_SDR_RX, 0, "VGA", vga_gain)
        self.sdr.setGain(SOAPY_SDR_RX, 0, "AMP", 14 if amp else 0)

        self.stream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        self.sdr.activateStream(self.stream)

    def set_frequency(self, freq_hz: float) -> None:
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, freq_hz)

    def read_samples(self, n: int, timeout_us: int = 200000) -> np.ndarray:
        buff = np.empty(n, dtype=np.complex64)
        got = 0
        while got < n:
            sr = self.sdr.readStream(self.stream, [buff[got:]], n - got, timeoutUs=timeout_us)
            if sr.ret <= 0:
                break
            got += sr.ret
        if got == 0:
            return np.empty(0, dtype=np.complex64)
        return buff[:got]

    def close(self) -> None:
        try:
            self.sdr.deactivateStream(self.stream)
            self.sdr.closeStream(self.stream)
        finally:
            self.stream = None


def dbfs_power(iq: np.ndarray) -> float:
    if iq.size == 0:
        return -200.0
    p = np.mean(np.abs(iq) ** 2)
    return 10.0 * math.log10(max(p, 1e-20))


def fm_demod(iq: np.ndarray) -> np.ndarray:
    if iq.size < 2:
        return np.empty(0, dtype=np.float32)
    phase = np.angle(iq[1:] * np.conj(iq[:-1]))
    return phase.astype(np.float32)


def video_likeness_score(iq: np.ndarray) -> float:
    """Cheap heuristic: analog FM video tends to have broad, structured baseband."""
    x = fm_demod(iq)
    if x.size < 4096:
        return -30.0

    x = x - np.mean(x)
    spec = np.fft.rfft(x * np.hanning(x.size))
    mag = np.abs(spec)
    if mag.size < 16:
        return -30.0

    freqs = np.fft.rfftfreq(x.size, d=1.0)
    # Heuristic in normalized frequency domain: favor wide energy spread.
    low = mag[(freqs > 0.01) & (freqs < 0.08)]
    mid = mag[(freqs >= 0.08) & (freqs < 0.25)]
    high = mag[(freqs >= 0.25) & (freqs < 0.45)]

    if low.size == 0 or mid.size == 0 or high.size == 0:
        return -30.0

    score = (
        6.0 * np.log10(np.mean(mid) + 1e-9)
        + 3.0 * np.log10(np.mean(low) + 1e-9)
        + 2.0 * np.log10(np.mean(high) + 1e-9)
    )
    return float(score)


def scan_frequencies(
    rx: HackRFReceiver,
    frequencies_hz: Sequence[float],
    dwell_ms: int,
    sample_rate: float,
    top_n: int,
) -> List[ScanResult]:
    n = max(4096, int(sample_rate * (dwell_ms / 1000.0)))
    results: List[ScanResult] = []

    for f in frequencies_hz:
        rx.set_frequency(f)
        time.sleep(0.01)
        iq = rx.read_samples(n)
        p = dbfs_power(iq)
        v = video_likeness_score(iq)
        results.append(ScanResult(freq_hz=f, power_db=p, video_score=v))
        print(f"{f/1e6:8.3f} MHz | power={p:7.2f} dB | video={v:7.2f} | score={p+v:7.2f}")

    ranked = sorted(results, key=lambda r: r.combined, reverse=True)
    return ranked[:top_n]


def make_linear_list(start_mhz: float, stop_mhz: float, step_mhz: float) -> List[float]:
    vals = []
    f = start_mhz
    while f <= stop_mhz + 1e-9:
        vals.append(f * 1e6)
        f += step_mhz
    return vals


def simple_video_reconstruct(
    demod: np.ndarray,
    sample_rate: float,
    standard: str,
    width: int,
    lines_per_frame: Optional[int] = None,
) -> Optional[np.ndarray]:
    if demod.size < 10000:
        return None

    if standard == "pal":
        line_rate = 15625.0
        default_lines = 625
    else:
        line_rate = 15734.0
        default_lines = 525

    lines = lines_per_frame or default_lines
    spl = int(sample_rate / line_rate)
    if spl < 10:
        return None

    needed = spl * lines
    if demod.size < needed:
        return None

    chunk = demod[:needed].reshape(lines, spl)

    # crude sync normalization (negative sync pulses)
    lo = np.percentile(chunk, 5)
    hi = np.percentile(chunk, 95)
    norm = (chunk - lo) / max(hi - lo, 1e-6)
    norm = np.clip(norm, 0.0, 1.0)

    resized = cv2.resize(norm, (width, lines), interpolation=cv2.INTER_LINEAR)
    img = (255.0 * resized).astype(np.uint8)
    return img


def watch_mode(
    rx: HackRFReceiver,
    freq_hz: float,
    sample_rate: float,
    standard: str,
    width: int,
) -> None:
    rx.set_frequency(freq_hz)
    print(f"Watching {freq_hz/1e6:.3f} MHz ({standard.upper()}). Press q to quit.")

    n = int(sample_rate * 0.08)
    if n < 20000:
        n = 20000

    title = f"FPV Preview {freq_hz/1e6:.3f} MHz"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)

    while True:
        iq = rx.read_samples(n)
        if iq.size < 2:
            continue
        demod = fm_demod(iq)
        img = simple_video_reconstruct(demod, sample_rate, standard, width)
        if img is not None:
            cv2.imshow(title, img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()


def choose_freqs_from_args(args: argparse.Namespace) -> List[float]:
    if args.preset == "58ghz":
        return sorted({f * 1e6 for f in FPV_58_PRESET_MHZ})
    return make_linear_list(args.start_mhz, args.stop_mhz, args.step_mhz)


def add_common_rx_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--sample-rate", type=float, default=20e6, help="RX sample rate in samples/s")
    p.add_argument("--bandwidth", type=float, default=20e6, help="RX analog filter bandwidth")
    p.add_argument("--lna-gain", type=float, default=32, help="HackRF LNA gain (0..40)")
    p.add_argument("--vga-gain", type=float, default=24, help="HackRF VGA gain (0..62)")
    p.add_argument("--amp", action="store_true", help="Enable HackRF RF amplifier")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="FPV detector/preview for HackRF")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_scan = sub.add_parser("scan", help="Scan frequencies and rank likely FPV channels")
    p_scan.add_argument("--start-mhz", type=float, default=0.0)
    p_scan.add_argument("--stop-mhz", type=float, default=6000.0)
    p_scan.add_argument("--step-mhz", type=float, default=4.0)
    p_scan.add_argument("--dwell-ms", type=int, default=40)
    p_scan.add_argument("--top", type=int, default=10)
    p_scan.add_argument("--preset", choices=["none", "58ghz"], default="none")
    add_common_rx_args(p_scan)

    p_watch = sub.add_parser("watch", help="Tune a frequency and preview analog video")
    p_watch.add_argument("--freq-mhz", type=float, required=True)
    p_watch.add_argument("--video-standard", choices=["pal", "ntsc"], default="pal")
    p_watch.add_argument("--width", type=int, default=1024)
    add_common_rx_args(p_watch)

    p_auto = sub.add_parser("auto", help="Scan then watch the best hit automatically")
    p_auto.add_argument("--start-mhz", type=float, default=0.0)
    p_auto.add_argument("--stop-mhz", type=float, default=6000.0)
    p_auto.add_argument("--step-mhz", type=float, default=4.0)
    p_auto.add_argument("--dwell-ms", type=int, default=40)
    p_auto.add_argument("--top", type=int, default=5)
    p_auto.add_argument("--preset", choices=["none", "58ghz"], default="58ghz")
    p_auto.add_argument("--video-standard", choices=["pal", "ntsc"], default="pal")
    p_auto.add_argument("--width", type=int, default=1024)
    add_common_rx_args(p_auto)

    return ap


def make_rx(args: argparse.Namespace) -> HackRFReceiver:
    bw = None if args.bandwidth <= 0 else args.bandwidth
    return HackRFReceiver(
        sample_rate=args.sample_rate,
        bandwidth=bw,
        lna_gain=args.lna_gain,
        vga_gain=args.vga_gain,
        amp=args.amp,
    )


def cmd_scan(args: argparse.Namespace) -> int:
    freqs = choose_freqs_from_args(args)
    print(f"Scanning {len(freqs)} frequencies...")
    rx = make_rx(args)
    try:
        top = scan_frequencies(rx, freqs, args.dwell_ms, args.sample_rate, args.top)
    finally:
        rx.close()

    print("\nTop candidates:")
    for i, r in enumerate(top, 1):
        print(
            f"{i:2d}. {r.freq_hz/1e6:8.3f} MHz | "
            f"power={r.power_db:7.2f} dB | video={r.video_score:7.2f} | score={r.combined:7.2f}"
        )
    return 0


def cmd_watch(args: argparse.Namespace) -> int:
    rx = make_rx(args)
    try:
        watch_mode(rx, args.freq_mhz * 1e6, args.sample_rate, args.video_standard, args.width)
    finally:
        rx.close()
    return 0


def cmd_auto(args: argparse.Namespace) -> int:
    freqs = choose_freqs_from_args(args)
    print(f"Auto mode: scanning {len(freqs)} frequencies...")

    rx = make_rx(args)
    try:
        top = scan_frequencies(rx, freqs, args.dwell_ms, args.sample_rate, args.top)
        if not top:
            print("No candidates found.")
            return 2
        best = top[0]
        print(f"\nBest candidate: {best.freq_hz/1e6:.3f} MHz")
        watch_mode(rx, best.freq_hz, args.sample_rate, args.video_standard, args.width)
    finally:
        rx.close()

    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.cmd == "scan":
        return cmd_scan(args)
    if args.cmd == "watch":
        return cmd_watch(args)
    if args.cmd == "auto":
        return cmd_auto(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
