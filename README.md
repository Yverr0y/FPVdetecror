# FPV Detector (HackRF One, Windows-friendly Python)

This repository contains a **starter tool** for detecting and viewing analog FPV drone video with a HackRF One.

## What it does

- Scans frequencies from **0 MHz to 6000 MHz** (configurable).
- Scores frequencies by RF energy and a simple analog-video heuristic.
- Shows a ranked list of likely FPV channels.
- Tunes to a selected frequency and attempts **real-time analog video preview**.
- Includes pre-built scan presets for common **5.8 GHz FPV bands**.

> ⚠️ This is a practical prototype. Analog FPV signals vary a lot, and robust decoding can still require fine tuning and better sync logic.

## OS / hardware support

- Tested conceptually for **Windows + HackRF One**.
- Uses **SoapySDR** with HackRF support (`SoapyHackRF`).

## Quick start (Windows)

1. Install HackRF drivers/tools (Zadig + official HackRF tools).
2. Install SoapySDR + SoapyHackRF.
3. Install Python 3.10+.
4. Install Python packages:

```powershell
py -m pip install -r requirements.txt
```

5. Run a wide scan:

```powershell
py fpv_detector.py scan --start-mhz 0 --stop-mhz 6000 --step-mhz 4
```

6. Tune and preview video:

```powershell
py fpv_detector.py watch --freq-mhz 5865 --video-standard pal
```

## Useful commands

Scan only typical analog FPV frequencies:

```powershell
py fpv_detector.py scan --preset 58ghz
```

Scan all + auto-watch strongest hit:

```powershell
py fpv_detector.py auto --start-mhz 0 --stop-mhz 6000 --step-mhz 4 --video-standard pal
```

Force NTSC timing:

```powershell
py fpv_detector.py watch --freq-mhz 5740 --video-standard ntsc
```

## Notes

- For best results, use a suitable antenna (5.8 GHz or broadband).
- Start with a smaller scan region around your expected band to improve speed.
- You can adjust gain, sample rates, and filters in the CLI flags.

## Legal

Use this tool only where monitoring is legal and authorized.
