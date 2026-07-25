# mmwave-cli

Linux driver and capture tools for the Texas Instruments **MMWCAS-RF-EVM**
(4× AWR2243 cascade, 77 GHz) and **MMWCAS-DSP-EVM** (TDA2xx) boards.

TI’s official path is Windows + mmWave Studio + MATLAB. This repo lets you
configure the radar and record raw IF ADC data (`.bin`) from Linux —
including Raspberry Pi.

Two capture front-ends share the same hardware:

| Path | Entry point | Config | Typical use |
|------|-------------|--------|-------------|
| **Python / Cython** | `mimo.py` | `radar_configs/*.toml` via `--radar-config` | Current workflow: raw IF capture |
| **Legacy C binary** | `./mmwave` (from `mimo.c`) | `config/*.toml` via `-f` / `--cfg` | Original CLI, simple record |

**Notes**
- Only Ethernet to the TDA is supported (default IP `192.168.33.180`, port `5001`).
- Raw captures land on the TDA SSD at `/mnt/ssd/<capture_dir>/` and are not
  auto-deleted — offload them with `fetch_to_usb.sh` or manual SSH/SCP.

---

## Hardware quick reference

| Item | Value |
|------|--------|
| Radar | TIDEP-01012 — 4× AWR2243 |
| TDA IP | `192.168.33.180` (static Ethernet) |
| TDA data path | `/mnt/ssd/<capture_dir>/` |
| SSH (from host) | `ssh -oHostKeyAlgorithms=+ssh-rsa -oPubkeyAcceptedAlgorithms=+ssh-rsa root@192.168.33.180` |

---

## Repository layout (relevant parts)

```
mmwave-cli/
├── mimo.py                  # Python capture (raw IF) — uses mmwcas + radar_configs/
├── mimo_interactive.py      # REPL wrapper around mimo.py — fixed frame count/session
├── mmwcas.pyx               # Cython bridge to TI mmWaveLink (C structs zeroed;
│                            #   all RF values come from TOML via mmw_set_config)
├── radar_config.py          # Loads radar_configs/*.toml
├── radar_configs/           # TOML presets for mimo.py
│   ├── cascade_tx3_rx16.toml   # default Python preset (3 TX chirps, Dev4 TX)
│   └── cascade_mimo.toml       # TI full 12-chirp / 4-device MIMO (from Lua)
├── fetch_to_usb.sh          # Offload captures from the TDA onto a USB drive
├── utility.py               # SCP helpers, .mmwave.json export, etc.
├── setup.py                 # Build mmwcas Cython extension
├── mimo.c / mimo.h          # Legacy C capture tool → builds ./mmwave
├── makefile                 # make all / make build / make install
├── config/                  # TOML presets for ./mmwave (C path)
│   ├── short-range-cfg.toml
│   └── ...
└── ti/                      # TI SDK / firmware (do not modify casually)
```

---

## 1. Python / Cython path (`mimo.py`)

This is the recommended path for raw IF capture on Linux / Raspberry Pi.

### Dependencies

```bash
pip install cython numpy pyserial
pip install tomli   # only needed on Python < 3.11 (stdlib tomllib on 3.11+)
```

### Build the `mmwcas` extension

```bash
make build-cython
# or
python3 setup.py build_ext --inplace
```

Verify:

```bash
python3 -c "import mmwcas; print('mmwcas OK')"
```

Full clean rebuild of C binary + Cython:

```bash
make build
```

### What `mimo.py` does

1. Loads a preset from `radar_configs/<name>.toml`
2. Calls `mmwcas.mmw_set_config()` then `mmw_init()`
3. Arms TDA → starts frames → waits N × framePeriodicity (+ 1 period) → stops
   (`--frames` programs radar `numFrames` + TDA `numberOfFramesToCapture`)
4. Writes `mmwave_json_files/<capture>.mmwave.json` and optionally IR timestamps
5. Uploads those sidecars into `/mnt/ssd/<capture_dir>/` next to the raw `.bin`s
6. Exits (single capture per process invocation)

**No built-in repeat-capture loop** (removed): running several captures
back-to-back within the same process - even with a real cooldown between
them - still showed capture sizes drifting smaller for identical `--frames`,
most likely an SSD/network throughput ceiling on the TDA (frame drops), not
something fixable host-side. For repeated captures, run `mimo.py` again
(fresh process each time) via a shell loop / cron, so you control spacing
and each capture gets a fully fresh `mmw_init()`.

### Typical usage (raw IF only)

```bash
# 100 frames (~10 s @ 100 ms), default radar preset (cascade_tx3_rx16)
python3 mimo.py --frames 100 --directory my_capture

# 300 frames ≈ 30 s at 10 fps
python3 mimo.py --frames 300 --directory my_capture --radar-config cascade_tx3_rx16

# TI full 12-chirp MIMO (from Cascade_Configuration_MIMO.lua)
python3 mimo.py --frames 100 --directory my_capture --radar-config cascade_mimo

# Disable IR GPIO timestamps
python3 mimo.py --frames 100 --no-ir
```

### CLI options (`mimo.py`)

| Option | Default | Description |
|--------|---------|-------------|
| `-d` / `--directory` | `mmwave_python` | Capture name prefix (timestamp appended) |
| `--frames` | `100` | Capture length in **radar frames** (exact) |
| `--tda-ip` | `192.168.33.180` | TDA IP |
| `--radar-config` | `cascade_tx3_rx16` | Preset name = `radar_configs/<name>.toml` |
| `--no-ir` | off | Disable IR timestamp logging |
| `--ir-pin` | `4` | BCM GPIO pin |
| `--ir-bounce-ms` | `200` | Debounce (ms) |

### `mimo_interactive.py` - REPL capture

Wrapper that reuses `mimo.py`'s capture logic (`run_one_capture()`) in a
loop: configure the radar once, then repeatedly type just an experiment name
at an `experiment>` prompt to arm + record, e.g. `bridge_test`. The frame
count is fixed for the whole session (set once via `--frames` at startup,
not overridable per prompt) so every capture is identical in length - same
TDA pre-allocation sizing, same arm/wait/stop timing - which lowers drop
risk versus varying it per capture. Blank/`quit`/`exit` stops.

Two hardware limitations remain, neither fixable host-side:
- Frame counts are wall-clock-based (RF chips stay at `numFrames=0`/infinite
  the whole session), not hardware-exact like `mimo.py`'s single-capture mode.
- Making many captures back-to-back in one long-lived process is exactly the
  scenario that showed SSD/network throughput drift (capture sizes shrinking
  run to run) - see `mimo.py`'s module docstring. Keep sessions reasonably
  short and watch for drift on long runs.

```bash
python3 mimo_interactive.py --frames 100
# experiment> bridge_test
# experiment> bridge_test_2
```

### Python-path TOML (`radar_configs/*.toml`)

**This is the only RF/geometry source for `mimo.py`.**  
`mmwcas.pyx` keeps zeroed C structs; missing TOML fields raise `ValueError`.

Preset name = filename without `.toml`:

| File | `--radar-config` | Meaning |
|------|------------------|---------|
| `radar_configs/cascade_tx3_rx16.toml` | `cascade_tx3_rx16` | **Default.** 3 chirps, Dev4 TX0/1/2, idle 175/7/7 µs, 4400 ksps, 255 loops, 100 ms frames |
| `radar_configs/cascade_mimo.toml` | `cascade_mimo` | TI Lua MIMO: 12 chirps, all 4 devices TX, slope 79, 8000 ksps, 64 loops, 10 frames |

#### Add a new preset

```bash
cp radar_configs/cascade_tx3_rx16.toml radar_configs/my_setup.toml
# edit my_setup.toml
python3 mimo.py --radar-config my_setup --frames 100
```

No Python edits required — files in `radar_configs/` are loaded automatically.

#### Schema overview (required sections)

```toml
[mimo.profile]
startFrequency = 77
frequencySlope = 60
idleTimes = [175, 7, 7]        # one idle time per profile slot (exactly 3)
adcStartTime = 6
numAdcSamples = 256
adcSamplingFrequency = 4400
rampEndTime = 65
rxGain = 48
# ... pfVcoSelect, HPF, backoff, etc.

[mimo.frame]
numLoops = 255
numFrames = 0                  # overridden by mimo.py --frames
framePeriodicity = 100
chirpStartIdx = 0
chirpEndIdx = 2
# ...

[mimo.channel]
rxChannelEn = 0x0F
txChannelEn = 0x07

[mimo.chirp]
numChirps = 3
profileIdPerChirp = [0, 1, 2]
txAntennaTable = [             # per device, local TX index per chirp (-1 = off)
    [-1, -1, -1],
    [-1, -1, -1],
    [-1, -1, -1],
    [0, 1, 2],
]
# ...

[mimo.adcOut]
[mimo.lowPower]
[mimo.misc]
[mimo.ldo]
[mimo.datapath]
# see cascade_tx3_rx16.toml for every required field
```

`config/*.toml` (C) and `radar_configs/*.toml` (Python) are **different schemas** —
do not mix them.

---

## 2. Legacy C binary (`./mmwave`)

### Build

```bash
sudo apt install build-essential
make all          # produces ./mmwave
# or
sudo make install # also copies to /usr/local/bin/mmwave
```

### Help

```bash
./mmwave -h
```

### Typical usage

```bash
# Configure + record (duration in MINUTES)
./mmwave -d outdoor0 --configure --record --time 10

# With a TOML config from config/
./mmwave -f config/short-range-cfg.toml --configure --record --time 2

# Interactive: configure once, then type experiment names at the prompt
./mmwave --configure --interactive -i 192.168.33.180
```

### CLI options (C binary)

| Option | Description |
|--------|-------------|
| `-c` / `--configure` | Program the RF / cascade |
| `-r` / `--record` | Arm TDA and record |
| `-t` / `--time` | Duration in **minutes** (default `1`) |
| `-d` / `--capture-dir` | Capture directory name on TDA SSD |
| `-i` / `--ip-addr` | TDA IP (default `192.168.33.180`) |
| `-p` / `--port` | TDA port (default `5001`) |
| `-f` / `--cfg` | Path to a TOML file under `config/` |
| `-I` / `--interactive` | Configure once, then prompt for capture names |

### C-path TOML (`config/*.toml`)

Used only by `./mmwave` / `mimo.c` (parsed by `toml/config.c`). Schema is the
**simpler single-profile** form, for example:

```toml
[mimo.profile]
id = 0
startFrequency = 77
frequencySlope = 79.0327
idleTime = 5
adcStartTime = 6
rampEndTime = 40
numAdcSamples = 256
adcSamplingFrequency = 8000
rxGain = 48

[mimo.frame]
numFrames = 0
numLoops = 16
framePeriodicity = 100

[mimo.channel]
rxChannelEn = 15
txChannelEn = 7
```

Examples live in `config/`. If `-f` is omitted, hardcoded defaults inside
`mimo.c` are used.

---

## 3. Fetching raw data from the TDA

Captures are **not** auto-deleted from the TDA's SSD after a capture, so pull
them off explicitly:

```bash
# Recommended: mount a USB drive and interactively pick a capture to copy
./fetch_to_usb.sh /dev/sda1

# Or manually:
# List captures
ssh -oHostKeyAlgorithms=+ssh-rsa -oPubkeyAcceptedAlgorithms=+ssh-rsa \
  root@192.168.33.180 'ls -lh /mnt/ssd'

# Copy one capture
scp -O -oHostKeyAlgorithms=+ssh-rsa -oPubkeyAcceptedAlgorithms=+ssh-rsa \
  -oStrictHostKeyChecking=no -r \
  root@192.168.33.180:/mnt/ssd/<capture_dir> ./
```

Each capture directory contains raw ADC `.bin` files (and, when using `mimo.py`,
the uploaded `.mmwave.json` / IR `.npy` sidecars).

---

## 4. Which tool should I use?

| Goal | Use |
|------|-----|
| Raw IF `.bin` only (recommended) | `python3 mimo.py --frames … --radar-config …` |
| Same, but legacy C CLI | `./mmwave -c -r -t … [-f config/….toml]` |
| Offload captures off the TDA | `./fetch_to_usb.sh /dev/sdXX` |

---

## 5. Config path cheat-sheet

| Front-end | TOML location | Select how |
|-----------|---------------|------------|
| `./mmwave` | `config/*.toml` | `-f config/short-range-cfg.toml` |
| `mimo.py` | `radar_configs/*.toml` | `--radar-config cascade_tx3_rx16` |

Built-in Python presets:

- **`cascade_tx3_rx16`** — default; 3 TX chirps on Dev4, 16 RX virtual array style geometry used in current experiments  
- **`cascade_mimo`** — full TI 12-chirp MIMO from `lua_reference/Cascade_Configuration_MIMO.lua`

---

## 6. Developer notes

- Do not casually edit `ti/mmwavelink` or `ti/firmware` (TI-provided).
- `ti/ethernet` and `ti/mmwave` are Linux ports of TI examples.
- C TOML parser: `toml/` (tomlc99). Python TOML loader: `radar_config.py` (`tomllib` / `tomli`).
- `mmwcas.pyx` must be rebuilt after Cython source changes: `make build-cython`.
- Changing RF/geometry for the Python path: edit `radar_configs/*.toml` only — no `.pyx` rebuild needed for field value changes.
