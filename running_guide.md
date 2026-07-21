# MIMO Radar Capture — Running Guide

## Table of Contents
- [Running mimo.c](#running-mimoc)
- [Running mimo.py](#running-mimopy)
- [Directory Structure](#directory-structure)
- [Important Notes](#important-notes)

---

## Running `mimo.c`

### 1. Dependency Setup
```bash
# Install build tools if not already installed
sudo apt install build-essential
```

### 2. Compile
```bash
gcc -o mimo mimo.c \
    -I./include \
    -L./lib \
    -lmmwavelink \
    -lmmwcas \
    -lpthread \
    -lm \
    -Wall
```
> Adjust the `-I` and `-L` paths to match the location of the TI mmWaveLink headers/library on your system.

### 3. Run
```bash
# Configure only
./mimo -c -i 192.168.33.180 -p 5001

# Record only (oneshot, 2 minutes)
./mimo -r -t 2.0 -d mmwl_capture -i 192.168.33.180 -p 5001

# Configure and record
./mimo -c -r -t 2.0 -d mmwl_capture -i 192.168.33.180 -p 5001

# Monitor mode (infinite loop, 10 second interval)
./mimo -c -r -m -n 10 -i 192.168.33.180 -p 5001
```

### `mimo.c` Argument List

| Argument | Description | Default |
|---|---|---|
| `-c` / `--configure` | Configure the radar | - |
| `-r` / `--record` | Start recording | - |
| `-t` / `--time` | Recording duration (minutes) | `1.0` |
| `-d` / `--capture-dir` | Capture directory name | `MMWL_Capture_<timestamp>` |
| `-i` / `--ip-addr` | TDA board IP | `192.168.33.180` |
| `-p` / `--port` | TDA board port | `5001` |
| `-m` / `--monitor` | Monitor mode (infinite loop) | - |
| `-n` / `--interval` | Interval between captures (seconds) | `10` |

---

## Running `mimo.py`

### 1. Dependency Setup
```bash
pip install cython
```

### 2. Build `mmwcas.pyx` (Cython)

First create the `setup.py` file:
```python
# setup.py
from setuptools import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize("mmwcas.pyx"),
)
```

Then build:
```bash
python setup.py build_ext --inplace
```
> On success, an `mmwcas.so` file (Linux) will appear in the same directory.

### 3. Run
```bash
# Record 10 seconds, 1 time (default)
python mimo.py

# Record 30 seconds with a custom directory
python mimo.py -d my_capture -t 30.0

# Record 3 loops, 60 second interval between loops
python mimo.py -d my_capture -t 10.0 -n 3 -i 60.0

# Infinite loop (Ctrl+C to stop)
python mimo.py -d my_capture -t 10.0 -n 0

# Change the TDA board IP
python mimo.py --tda-ip 192.168.33.180

# Interactive mode: configure once, then repeatedly type an experiment name
# at the prompt (mirrors `mimo.c --interactive`) - no reconfiguration
python mimo.py --interactive

# Disable IR sensor timestamp logging (e.g. testing without GPIO/sensor)
python mimo.py --no-ir

# Change the IR sensor's GPIO pin (BCM) and debounce
python mimo.py --ir-pin 4 --ir-bounce-ms 200
```

> **Radar geometry note**: `mimo.py` is PATCHED to be identical to `mimo.c`
> (3 chirps, 1 TX device, 3 profiles with idle time 175us/7us/7us, 60MHz/us
> slope, 65us rampEnd, 255 loops/frame, 100ms/10Hz frame periodicity). See
> the header comment in `mimo.py` for details.

### `mimo.py` Argument List

| Argument | Description | Default |
|---|---|---|
| `-d` / `--directory` | Capture directory name | `mmwave_python` |
| `-t` / `--duration` | Recording duration (seconds) | `10.0` |
| `--tda-ip` | TDA board IP | `192.168.33.180` |
| `-n` / `--num-loops` | Number of loops (0 = infinite) | `1` |
| `-i` / `--inter-loop-time` | Delay between loops (seconds) | `60.0` |
| `-I` / `--interactive` | Configure once, then loop on experiment names typed at the prompt | - |
| `--no-ir` | Disable IR sensor timestamp logging | - |
| `--ir-pin` | IR sensor GPIO pin (BCM) | `4` |
| `--ir-bounce-ms` | IR sensor software debounce (ms) | `200` |

> **IR sensor timestamp logging** (built-in, replaces the now-removed
> `run_experiment.sh` + `ir_logger.py`): GPIO rising-edge detection is armed
> once at startup, then timestamp recording is switched on/off precisely
> around each capture's TDA framing window (in `run_one_capture()`). Output
> is saved to `ir_timestamps/<capture_dir>_ir_timestamps.npy` (float64 array
> of Unix epoch seconds). Automatically disabled (with a printed notice) if
> `RPi.GPIO` isn't available (e.g. running off a Raspberry Pi).
>
> After a successful capture, this `.npy` file plus the capture's
> `.mmwave.json` are both SCP'd up into `/mnt/ssd/<capture_dir>/` on the TDA
> board itself, alongside the raw `.bin` data, so they travel together
> through the normal SCP-download step later (e.g. `pipeline.py`).

---

## Directory Structure

```
project/
├── mimo.c
├── mimo.py
├── mmwcas.pyx
├── setup.py
├── utility.py
├── include/            ← TI headers (mmwave.h, etc.)
├── lib/                ← TI libraries (.so / .a)
├── mmwave_json_files/  ← generated JSON output (created automatically)
└── ir_timestamps/      ← generated IR sensor timestamp output per capture (created automatically)
```

---

## Important Notes

- Make sure the TDA board is **powered on and connected to the network** before running the script.
- Make sure SSH access to `root@192.168.33.180` works **without a password** (using an SSH key), since `utility.py` uses SSH to verify captured files.
- Captured files are stored at `/mnt/ssd/<capture_dir>` on the TDA board.
- The `.mmwave.json` file is stored in the `mmwave_json_files/` directory on the host.
