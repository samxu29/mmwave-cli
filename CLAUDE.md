# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Capture tooling for **Bridge Structural Health Monitoring** research using a TIDEP-01012 MIMO Cascade Radar (77 GHz, 4-chip AWR2243 cascade). `mimo.py` configures the radar, arms the TDA cascade board, and records raw IF ADC data to the TDA's SSD; captures are pulled off onto a USB drive with `fetch_to_usb.sh` for offline processing.

**Research context:** PhD project at IMRSL, Muroran Institute of Technology.  
**Goal:** Detect bridge dominant vibration frequency and displacement via sub-mm radar (Permanent Scatterer) interferometry.

**Status:** Raw IF capture (`mimo.py` / `mimo_interactive.py`) is the active workflow. Mid-capture frame drops track the radar RF/geometry preset (`--radar-config`), not the CLI arm path — see the FRAME DROPS section in `mimo.py`'s module docstring (`cascade_baseline.toml` is a known-clean reference).

---

## Build & Setup

### Python dependencies (Raspberry Pi)
```bash
pip install cython pyserial numpy scipy matplotlib
pip install tomli  # only needed on Python < 3.11 (stdlib tomllib covers 3.11+);
                   # used by radar_config.py to load radar_configs/*.toml
```

### Build the `mmwcas` Cython extension
```bash
make build                         # full clean + rebuild
python setup.py build_ext --inplace  # Cython only (faster)
```
After a successful build, `mmwcas.cpython-*.so` appears in the repo root.

### Verify the build
```bash
python3 -c "import mmwcas; print('mmwcas OK')"
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Radar | TIDEP-01012 — 4× AWR2243, 12 TX × 16 RX virtual, 77 GHz |
| DSP board | TDA2XX (mmWave DSP EVM) |
| TDA IP | `192.168.33.180` (static, over Ethernet) |
| TDA data storage | `/mnt/ssd/<capture_dir>/` (not auto-deleted — offload with `fetch_to_usb.sh` or SSH/SCP manually) |
| Processing host | Raspberry Pi 5 (ARM Cortex-A76), hostname `imrslpi5-02`, user `imrsl`, password `imrsl2022` |

**Key radar parameters (`mimo.py` `config_dict`) — PATCHED to match `mimo.c` exactly:**
- `framePeriodicity = 100 ms` → frame rate = 10 Hz (was 50ms/20Hz — halves the max
  detectable structural vibration frequency to ~5 Hz by Nyquist)
- `numAdcSamples = 256`, `adcSamplingFrequency = 4400 ksps` (corrected 2026-07-22 to match AWR1843 reference `SAMPLE_RATE`)
- `frequencySlope = 60 MHz/μs`, `rampEndTime = 65 μs`
- `numLoops = 255` chirp loops per frame (was 16)
- 3 chirps only (not 12), single TX device (TX0/TX1/TX2 on chirp0/1/2), 3 profiles
  with idle time 175μs/7μs/7μs (fixed, not configurable via `config_dict`)
- `mimo.py` runs exactly one capture per process invocation, then exits (no
  built-in loop - repeated same-process captures showed SSD/network
  throughput drift; use an external shell loop instead).
  The old `-I`/`--interactive` REPL mode (configure once, loop on typed
  experiment names) has moved to `mimo_interactive.py`. Frame count is
  fixed for the whole session (set once via `--frames` at startup, not
  overridable per prompt) to keep TDA pre-allocation sizing and wait
  timing identical capture to capture, lowering drop risk — each prompt
  takes only an experiment name. Same hardware caveats as before apply:
  wall-clock-based frame counts (RF chips stay at `numFrames=0`/infinite)
  and SSD/network throughput drift risk on long back-to-back-capture
  sessions within one process.
- IR sensor timestamp logging is built directly into `mimo.py` (replaces the old
  `run_experiment.sh` + `ir_logger.py` signal-based glue, both removed): GPIO
  rising-edge detection (BCM pin 4 default, 200ms debounce, same convention as
  `receiver_ir.py`) is armed once at startup, and recording is gated on/off
  around each capture's TDA framing window in `run_one_capture()`. Output:
  `ir_timestamps/<capture_dir>_ir_timestamps.npy` (float64 array of Unix epoch
  seconds, same convention as `receiver_ir.py`'s `gpio_timestamps.npy`).
  Disable with `--no-ir`; override pin/debounce with `--ir-pin`/`--ir-bounce-ms`.
  Silently disabled with a printed notice if `RPi.GPIO` isn't importable (e.g. off-Pi).
- After a successful capture, `run_one_capture()` SCPs the local IR timestamps
  `.npy`, both per-frame timestamp sidecars (below), and `.mmwave.json` up
  into `/mnt/ssd/<capture_dir>/` on the TDA (`utility.upload_files_to_tda()`),
  so they sit alongside the raw `.bin` data and travel together whenever the
  capture is offloaded (e.g. `fetch_to_usb.sh`).
- `run_one_capture()` always resyncs the TDA's system clock to the host's over
  SSH before arming (`utility.sync_tda_clock()`) — the TDA is on an isolated
  subnet with no real time source, so its RTC free-runs from an arbitrary
  boot value (observed years off, e.g. stuck at 2019). This keeps TDA-side
  OS-clock artifacts (capture directory file mtimes, etc.) meaningful.
  **It does NOT make `<dev>_0000_idx.bin`'s per-frame timestamp comparable
  to the IR sensor's host-clock timestamps** — verified empirically that
  field is a separate monotonic counter (microseconds since some
  boot/driver-init reference, nowhere near an epoch value), unaffected by
  the OS date (an earlier version of this doc claimed otherwise). See the
  two per-frame timestamp sidecars below instead.
  The TDA's `date` is BusyBox v1.30.1, confirmed live via `date --help`
  (also confirmed by `%N` coming back as a literal, unexpanded string); the
  set step tries the two `-s TIME` forms BusyBox's own `--help` documents
  (`YYYY-MM-DD hh:mm:ss` and positional `YYYYMMDDhhmm.ss`) rather than
  GNU's `-s @epoch` shorthand. Caps precision at roughly 1-2s (whole
  seconds + one SSH round trip). No opt-out — every capture path
  (`mimo.py`, `mimo_interactive.py`) goes through `run_one_capture()`, so
  this always runs.
- `run_one_capture()` also fetches that capture's `*_idx.bin` and exports two
  per-frame timestamp sidecars (one entry per frame ACTUALLY captured, so
  drops are reflected naturally):
  - `rf_frame_timestamps/<capture_dir>_rf_frame_timestamps.npy` — the RF/DSP
    side's own per-frame timestamp (`utility.fetch_rf_frame_timestamps()`),
    TDA monotonic clock, **not** wall-clock.
  - `rpi_frame_timestamps/<capture_dir>_rpi_frame_timestamps.npy` — this
    host's estimate of when each of those same frames occurred, ON THIS
    HOST'S OWN CLOCK (`utility.save_rpi_frame_timestamps()`), built by
    anchoring the RF timestamps' relative spacing to `time.time()` at the
    moment `mmw_start_frame()` returned. Directly comparable to the IR
    sensor timestamps (also host clock) — the RF sidecar is not.
- `check_timestamp.py` sanity-checks a capture's IR marker AND radar frame
  regularity — the rig spins at a constant rev/s, so marker-to-marker (and
  frame-to-frame) intervals should be very uniform; it flags intervals
  >1.5x median (missed trigger / dropped frame) or <0.5x median
  (debounce/double-trigger), reports measured vs `--expected-rps`, and
  cross-checks the IR marker span's *duration* against the radar's own
  frame timestamp span. It also reports the nearest-frame time difference
  for every IR marker — an exact comparison against
  `rpi_frame_timestamps.npy` when available, or an approximate one against
  `rf_frame_timestamps.npy`/`idx.bin` otherwise (see the caveat above).
  Supports `--fetch <capture_dir>` to pull just the small sidecars from the
  TDA directly, same pattern as `parse_idx.py --fetch`.

---

## Repository Structure

```
mmwave-cli/                         ← this repo (runs on Raspberry Pi ~/mmwave-cli/)
├── mimo.py                         ← radar control: configure, arm, capture, stop
├── mimo_interactive.py             ← REPL wrapper around mimo.py — fixed frame count/session, name-only prompts
├── radar_config.py                 ← loads RF/geometry presets from radar_configs/*.toml
├── radar_configs/                  ← RF/geometry presets (TOML), select via --radar-config <name>
│   └── default.toml
├── mimo.c / mimo.h                 ← standalone C binary (legacy/alternate path)
├── mmwcas.pyx / mmwcas.pyi         ← Cython wrapper — compiled to mmwcas.so (Python import)
├── setup.py                        ← build mmwcas Cython extension
├── makefile                        ← build C binary + Cython extension
├── fetch_to_usb.sh                 ← offload captures from the TDA onto a USB drive
├── utility.py                      ← check_captured_files(), upload_files_to_tda(), export_config_to_json(), signal_handler()
├── mmwave_json_files/              ← generated .mmwave.json config files (per capture, moved into capture dir after SCP)
├── ir_timestamps/                  ← generated IR sensor timestamp files (per capture, see mimo.py IR logging)
└── ti/                             ← TI SDK headers and firmware (do not modify)
```

---

## Known Issues & Fixes

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| Only 6–10 frames per 10 s capture | No `time.sleep()` between start/stop | Added `time.sleep(args.duration)` in mimo.py |

---

## SSH/SCP Access

```bash
# SSH to Raspberry Pi
ssh imrsl@imrslpi5-02           # password: imrsl2022

# SSH to TDA board (from Raspberry Pi)
ssh -oHostKeyAlgorithms=+ssh-rsa -oPubkeyAcceptedAlgorithms=+ssh-rsa \
    -oStrictHostKeyChecking=no root@192.168.33.180

# Manual SCP from TDA (or use fetch_to_usb.sh to copy onto a mounted USB drive)
scp -O -oHostKeyAlgorithms=+ssh-rsa -oPubkeyAcceptedAlgorithms=+ssh-rsa \
    -oStrictHostKeyChecking=no -r \
    root@192.168.33.180:/mnt/ssd/<capture_dir> ./
```

---

## Capture Directory Naming

Format: `<label>_<YYMMDD>_<HHMMSS>`  
Example: `RPI_python_bridge_260525_080012`

The `--exp-name` argument sets the prefix. Timestamp is appended automatically by `mimo.py`.

---

## Planned Next Steps

- [ ] **Corner reflector** — trihedral CR at midspan, ~30–45° elevation angle from riverbank
