"""
mimo.py - TIDEP-01012 MIMO Cascade Radar Control (Python / mmwcas Cython wrapper)

RF/geometry configuration (chirp profiles, idle times, antenna geometry,
frame timing) has moved OUT of this file and into radar_config.py
(2026-07-22) - see that module's header for details and for how to add a new
idle-time scheme or antenna geometry preset without editing this file. Select
a non-default preset with --radar-config <name>.

FRAME DROPS (investigated 2026-07-24, bug/capture-frame-drop - READ THIS BEFORE
RE-INVESTIGATING). frame_sweep.py swept capture length 25..600 frames in
randomised order and showed the shortfall is TWO SEPARATE mechanisms, which is
why single-length measurements looked so noisy:

  (A) A flat ~2 frames lost at EVERY length, with zero internal gaps in the
      index - i.e. tail/edge truncation, not mid-capture loss. Independent of N,
      so it is 6% of a 25-frame capture and 0.3% of a 600-frame one. This is the
      whole story for short captures. TDA_ARM_FRAME_HEADROOM does not fix it.

  (B) A handful of internal mid-capture gaps, single frames absent from an
      otherwise perfect grid (each gap exactly 2.00 periods, survivors at median
      100.01 ms) at IDENTICAL indices on master and slave - so a shared cause
      upstream of the per-device links. Typically 1-5% of frames, with large
      run-to-run scatter. Position within the capture is NOT fixed: clean runs
      have started dropping only after ~30 s, but a probed 600-frame run lost
      its first frame at t+0.8 s and kept dropping throughout - so "late onset"
      is a common pattern, not a law.

HOW (B) IS NOT CAUSED - settled by direct observation (tda_probe.py,
probe_run_260724_211910, 567/600 frames, 28 internal gaps):
  * Page cache / dirty_expire. Dirty and Writeback stayed 0.0 MB for the entire
    capture while the SSD wrote a steady ~55-66 MB/s. The capture path bypasses
    the page cache (O_DIRECT or equivalent). vm.dirty_expire_centisecs = 3000
    matching the ~30 s onset on some runs was a coincidence; tuning vm.dirty_*
    cannot help.
  * SSD saturation. Disk busy was 10-22% during the gap phase, never pegged,
    and write MB/s did not collapse when frames were lost.
  * Write rate / bandwidth. 600 frames at 100 ms (62.7 MB/s) vs 200 ms
    (31.3 MB/s) lost a similar number of internal frames.
  * RF frame overrun / duty cycle, SSD capacity, temperature, host wait,
    session drift - see earlier experiments.

PRIME SUSPECT now is our CLI / Ethernet arming path, NOT the TDA hardware or
the RF config. Studio A/B at the same cascade_tx3_rx8 geometry and 600 frames
(lua_reference/Cascade_Configuration_Capture_cascade_tx3_rx8_600.lua,
studio_tx3_rx8_600_20260724_212505):
  * Studio: 599/600, ZERO internal gaps, perfect 100.01 ms median grid, only a
    clean 1-frame tail shortfall (= mechanism A).
  * CLI:    routinely 5-33 internal mid-capture gaps on the same length.
So the RF schedule, SSD, and TDA capture firmware can deliver a clean 60 s
capture when Studio drives them. Mechanism (B) is introduced by how we
configure/arm/stop from mmwcas on the Pi. Remaining bisect targets: TDA
numberOfFramesToCapture (Studio passes 0 = follow FrameConfig; we pass N+50),
post-arm settle, CreateApp / ConnectTDA sequencing, RfInitCalibConfig, and
stop/de-arm behaviour.

WHAT HELPED, MODESTLY: arming with numberOfFilesToAllocate > 0 so the TDA is not
extending the capture file while frames stream in. Pooled means over identical
300-frame runs - OFF 286.5 (280/284/290/292), ON 291.8 (286/290/291/293/299): ~5
frames better and a higher floor, but well inside run-to-run scatter, so treat it
as an improvement rather than a fix. _tda_prealloc_files() sizes it;
--tda-prealloc-files 0 restores the old behaviour for A/B testing. Because TI
fixes each pre-allocated file at 2047 MB regardless of frames recorded, the
padding is trimmed back to the real payload afterwards (--no-reclaim-padding to
keep it).

CONSEQUENCE FOR DOWNSTREAM - the important part. Frame counts come from
<dev>_0000_idx.bin (24-byte header + 48 bytes/frame), never from the data file
size, which pre-allocation inflates. That index also stores a TIMESTAMP per
frame, and processing MUST place frames by those timestamps: treating N captured
frames as N contiguous samples at dt scales every derived frequency by the drop
fraction (at 283/300 that is a 5.5% bias, ~0.11 Hz on a 2 Hz mode - larger than
the 0.067 Hz frequency resolution, and it varies per capture). Displacement is
far less affected, since a 200 ms gap needs >lambda/4 ~ 0.97 mm of motion to
break phase unwrapping. Use parse_idx.py to inspect any capture.

Capture length is defined by FRAME COUNT (--frames), not wall-clock seconds.
The radar + TDA are programmed with that numFrames so they stop after exactly
N frames (e.g. 300 frames @ 100 ms period ≈ 30 s). Host sleep is only
N × framePeriodicity + a small margin so we do not stop_frame early.

Single capture per process invocation, then exit. For repeats, re-invoke
this script (fresh mmw_init() each time) from a shell loop or cron - do not
add a capture loop back into this file (see the "Single capture, then exit"
comment in main() for why).

The old --interactive REPL mode (configure once, loop on typed experiment
names) has moved to mimo_interactive.py and is DEPRECATED - see that file's
docstring.

IR sensor timestamp logging (replaces the old run_experiment.sh + ir_logger.py
signal-based glue) is now built in: GPIO edge detection is armed once at
startup (same BCM pin/RISING-edge/200ms-debounce convention as ir_logger.py /
receiver_ir.py), and timestamp collection is gated on/off directly around
each capture's TDA framing window in run_one_capture() - no subprocess, no
signals. Output: ir_timestamps/<capture_dir>_ir_timestamps.npy, a float64
array of Unix epoch seconds (same convention as receiver_ir.py's
gpio_timestamps.npy). Silently disabled (with a printed notice) if
RPi.GPIO isn't importable or the pin can't be armed (e.g. running off-Pi),
or if --no-ir is passed.

After a successful capture, the local IR timestamps .npy and the .mmwave.json
config file are both SCP'd up into /mnt/ssd/<capture_dir>/ on the TDA board
itself (alongside the raw .bin data), so they travel with the capture as a
single unit for whatever downstream transfer step (e.g. fetch_to_usb.sh)
pulls the data off the TDA - no separate correlation step needed.
"""
import time
import argparse
import copy
from datetime import datetime
import sys
import mmwcas
import signal
import numpy as np
from utility import export_config_to_json
from utility import check_captured_files
from utility import signal_handler
from utility import upload_files_to_tda
from utility import truncate_capture_padding
from utility import read_tda_thermal
from utility import sync_tda_clock
from utility import fetch_frame_timestamps
from utility import TeeLogger
from radar_config import RADAR_CONFIGS, DEFAULT_RADAR_CONFIG, get_radar_config
import os


def _frame_period_s(cfg):
    """Frame period in seconds from a radar config dict (periodicity is ms)."""
    return float(cfg["mimo"]["frame"]["framePeriodicity"]) / 1000.0


# Extra frames to arm the TDA with, on top of the N we actually want.
#
# The TDA's recording window is bounded by the numberOfFramesToCapture we arm
# it with, and that window opens at ARM time - not when the master emits its
# first frame, which is ~2.1 s later (post-arm settle plus the slaves-then-
# master sequencing in mmw_start_frame()). A TDA armed for exactly N can
# therefore close while the radar is still emitting the tail.
#
# SCOPE: this only fixes the small TAIL deficit, which parse_idx.py measured at
# ~2 frames of a 20-frame shortfall. It is NOT the main frame-drop cause - the
# other ~18 were mid-capture skips (see the note in _warn_on_frame_drops).
# Cheap insurance, not a cure; do not read a clean 300 here as proof the drop
# bug is solved without checking parse_idx.py for internal gaps.
#
# Arming with headroom costs nothing: the RF chips are still programmed with
# frame.numFrames = N and self-stop after exactly N frames, so the extra
# allowance is never filled - it only stops the TDA closing the window early.
# The host wait below still bounds the session, and mmw_dearming_tda() ends
# recording either way.
TDA_ARM_FRAME_HEADROOM = 50


def _wait_s_for_frames(num_frames, period_s):
    """Host wait after start_frame before de-arming the TDA.

    = N periods (the actual framing time) + a startup/flush margin.

    The margin covers the latency between start_frame() returning and the
    master ACTUALLY emitting its first frame (slave-arm sleep, RPC round-trips,
    first-frame calibration, PLL settle) PLUS TDA flush at the end. Necessary
    but NOT sufficient on its own: raising it from one period to 5 s did not
    recover any frames, because the binding constraint is the TDA's own
    arm-time window, not this sleep - see TDA_ARM_FRAME_HEADROOM above. Keep
    it generous anyway so de-arm never cuts off a still-running radar; the
    radar auto-stops at numFrames so waiting extra is harmless (stop_frame()
    then just reports frame-already-ended)."""
    return num_frames * period_s + period_s + 5.0


def _rx_popcount_per_device(cfg):
    """Return [nRx0, nRx1, nRx2, nRx3] from mimo.channel.rxChannelEn
    (scalar broadcasts to all four devices; list is per-device)."""
    rx = cfg["mimo"]["channel"]["rxChannelEn"]
    if isinstance(rx, (list, tuple)):
        return [bin(int(x)).count("1") for x in rx]
    n = bin(int(rx)).count("1")
    return [n, n, n, n]


def _data_packing(cfg):
    """TDA dataPacking mode for this capture: 0 = 16-bit, 1 = packed 12-bit.
    Read from mimo.datapath.dataPacking, which mimo.py sets from
    --data-packing; absent means 16-bit."""
    try:
        return 1 if int(cfg["mimo"]["datapath"].get("dataPacking", 0)) == 1 else 0
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0


def _expected_bytes_per_frame_for_device(cfg, dev_id):
    """
    On-disk bytes for ONE frame in a single device's <dev>_0000_data.bin.

        real samples/frame/device =
            numAdcSamples × chirpsPerLoop × numLoops × numRxOnThisDevice × 2 (I+Q)

    times bytes per sample, which depends on the TDA packing mode: 2 bytes at
    16-bit (dataPacking 0), or 1.5 bytes at packed 12-bit (dataPacking 1, four
    samples per 6 bytes) - computed as samples*3//2 to stay exact.

    chirpsPerLoop = chirpEndIdx - chirpStartIdx + 1. numRxOnThisDevice is the
    popcount of that device's rxChannelEn (supports per-device lists). Returns
    0 when that device has RX disabled; None if fields are missing/unparseable
    (frame-drop check is then silently skipped).
    """
    try:
        prof = cfg["mimo"]["profile"]
        frame = cfg["mimo"]["frame"]
        num_adc = int(prof["numAdcSamples"])
        num_loops = int(frame["numLoops"])
        chirps_per_loop = int(frame["chirpEndIdx"]) - int(frame["chirpStartIdx"]) + 1
        num_rx = _rx_popcount_per_device(cfg)[dev_id]
        if num_adc <= 0 or num_loops <= 0 or chirps_per_loop <= 0:
            return None
        if num_rx <= 0:
            return 0
        samples = num_adc * chirps_per_loop * num_loops * num_rx * 2
        if _data_packing(cfg) == 1:
            return samples * 3 // 2
        return samples * 2
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def _frame_active_time_us(cfg):
    """
    Total time the radar spends chirping in ONE frame, in microseconds:

        active = numLoops × Σ_chirps (idleTime[profile] + rampEndTime)

    This must fit inside framePeriodicity (ms) or the device can't honour the
    frame trigger and SKIPS frames (RF frame overrun) - a data-rate-independent
    drop cause. Returns None if fields are missing/unparseable.
    """
    try:
        prof = cfg["mimo"]["profile"]
        frame = cfg["mimo"]["frame"]
        chirp = cfg["mimo"]["chirp"]
        idle_times = prof["idleTimes"]
        ramp = float(prof["rampEndTime"])
        num_loops = int(frame["numLoops"])
        pids = chirp["profileIdPerChirp"]
        per_loop = sum(float(idle_times[int(pid)]) + ramp for pid in pids)
        return per_loop * num_loops
    except (KeyError, TypeError, ValueError, IndexError):
        return None


# Each file the TDA pre-allocates is a fixed 2047 MB (TI's Cascade_Capture.lua).
TDA_PREALLOC_FILE_BYTES = 2047 * 1024 * 1024

# Sustained sequential write the TDA's NVMe actually delivers, MB/s (10^6 B).
# Measured 2026-07-24 by writing 3 GB in 500 MB chunks with a sync after each:
# 92.1, 101.5, 93.2, 95.7, 91.7, 91.6 MB/s - flat, so no SLC-cache exhaustion or
# thermal throttling, just a modest ceiling. Far below what "NVMe" implies, and
# low enough that a capture's aggregate write rate has to be checked against it.
TDA_SSD_WRITE_MBPS = 92.0

# Fraction of TDA_SSD_WRITE_MBPS above which a capture is considered to have too
# little headroom to absorb a stall without losing frames.
TDA_SSD_WRITE_WARN_FRACTION = 0.7

# Shortfall (frames) reported as a one-line note rather than the full diagnosis.
# Deliberately small: this rig's baseline loss is ~3-5% (see FRAME DROPS above),
# so most captures DO print the long form. That is intended - the shortfall is
# real, it is not yet fixed, and downstream has to know the frames are missing.
# Raising this would only hide it.
FRAME_SHORTFALL_NOISE_FLOOR = 2

# Bytes of the TDA's <dev>_0000_idx.bin: a 24-byte file header followed by one
# 48-byte entry per frame the TDA ACTUALLY captured. That entry count is the
# authoritative frame count - unlike the data file it can't be inflated by
# pre-allocation padding. See parse_idx.py for the full struct layout.
_IDX_HEADER_BYTES = 24
_IDX_ENTRY_BYTES = 48


def _tda_prealloc_files(cfg, num_frames):
    """How many files to have the TDA pre-allocate for this capture.

    Zero pre-allocation (the previous hard-coded behaviour) makes the TDA grow
    the capture file as frames stream in, and that filesystem extension work
    lands on the write path - the measured result was random single frames
    vanishing from an otherwise perfect frame grid, identically on every
    device, at a rate independent of data rate (so neither a bandwidth ceiling
    nor RF scheduling). TI documents this field as the fix in
    Cascade_Capture.lua: "the number of files to preallocate on the SSD. This
    improves capture reliability by not having frame drops while switching
    files. The tradeoff is that each file is a fixed 2047 MB even if a smaller
    number of frames are captured."

    Sized off the LARGEST per-device stream (each device writes its own file),
    rounded up, floor of 1. Returns 1 if the per-frame size is unknown.
    """
    per_dev = [_expected_bytes_per_frame_for_device(cfg, d) for d in range(4)]
    bpf = max((b for b in per_dev if b), default=None)
    if not bpf:
        return 1
    total = bpf * max(int(num_frames), 1)
    return max(1, -(-total // TDA_PREALLOC_FILE_BYTES))   # ceil division


def _required_write_rate_mbps(cfg):
    """Aggregate MB/s (10^6 B) the TDA must write to keep up with this preset.

    Summed across all RX-enabled devices, since they all land on the same SSD:
        sum(bytes/frame/device) / framePeriodicity
    Returns None if the per-frame size or frame period can't be derived.
    """
    try:
        period_s = float(cfg["mimo"]["frame"]["framePeriodicity"]) / 1000.0
    except (KeyError, TypeError, ValueError):
        return None
    if period_s <= 0:
        return None
    total = 0
    for dev in range(4):
        bpf = _expected_bytes_per_frame_for_device(cfg, dev)
        if bpf:
            total += bpf
    if not total:
        return None
    return total / period_s / 1e6


def _report_write_budget(cfg):
    """Print the capture's write rate against what the TDA SSD can sustain.

    Worth checking before every capture: at ~92 MB/s measured, a 4-device 16-bit
    preset at 10 fps needs ~126 MB/s, which is arithmetically impossible to
    sustain (short captures survive only while TDA DDR absorbs the deficit).

    Staying under budget is necessary but NOT sufficient - cascade_tx3_rx8 sits
    at 68% and still drops ~3-5%, and cutting it to 47 MB/s with 12-bit packing
    made the loss worse. So read a green figure here as "bandwidth is not your
    problem", never as "this capture will be complete". Returns the rate or None.
    """
    req = _required_write_rate_mbps(cfg)
    if req is None:
        return None
    pct = req / TDA_SSD_WRITE_MBPS * 100.0
    packing = "12-bit packed" if _data_packing(cfg) == 1 else "16-bit"
    print(f"TDA write budget : {req:.1f} MB/s needed, "
          f"{TDA_SSD_WRITE_MBPS:.0f} MB/s measured ({pct:.0f}%)  [{packing}]")
    if req > TDA_SSD_WRITE_MBPS:
        print(f"  ** OVER BUDGET: the SSD cannot keep up. Expect heavy frame "
              f"loss on longer captures")
        print(f"     (short ones survive only while TDA DDR absorbs the "
              f"deficit). Reduce numLoops /")
        print(f"     RX count / numAdcSamples, or raise framePeriodicity.")
    elif req > TDA_SSD_WRITE_MBPS * TDA_SSD_WRITE_WARN_FRACTION:
        print(f"  NOTE: little headroom - a brief SSD stall can't be absorbed, "
              f"so occasional")
        print(f"  single-frame losses are likely. Check with parse_idx.py.")
    return req


def _frames_from_idx_size(size):
    """Frames the TDA indexed, from an <dev>_0000_idx.bin byte size.
    Returns None if the size can't hold a header + at least one entry."""
    body = int(size) - _IDX_HEADER_BYTES
    if body < _IDX_ENTRY_BYTES:
        return None
    return body // _IDX_ENTRY_BYTES


def _dev_id_from_data_bin(name):
    """Parse leading device index from e.g. 'master_0000_data.bin' /
    'slave1_0000_data.bin' / '0_0000_data.bin'. Returns None if unknown."""
    base = str(name).split("/")[-1].lower()
    if base.startswith("master"):
        return 0
    if base.startswith("slave1") or base.startswith("slave_1"):
        return 1
    if base.startswith("slave2") or base.startswith("slave_2"):
        return 2
    if base.startswith("slave3") or base.startswith("slave_3"):
        return 3
    # Numeric prefix used by some TDA layouts: '0_0000_data.bin'
    head = base.split("_", 1)[0]
    if head.isdigit():
        return int(head)
    return None


def _read_tda_temps(label, tda_ip):
    """Print the TDA2 SoC thermal zones, return them. Read OUTSIDE the armed
    window (before arming / after de-arming) so the SSH login can't add load
    while the TDA is capturing."""
    zones = read_tda_thermal(tda_ip)
    if not zones:
        return None
    hottest = max(zones.values())
    detail = "  ".join(f"{k} {v:.1f}" for k, v in sorted(zones.items()))
    print(f"    [TDA TEMP] {label:5s} max {hottest:.1f}C   ({detail})")
    return zones


def _read_temps(label):
    """Print on-die temperatures for every active device, return the reading.

    Sampled around a capture (never during framing) to test the thermal
    explanation for mid-capture frame loss: drops begin partway in and start
    earlier on each successive run, consistent with each run beginning hotter.
    Tolerates an mmwcas built before mmw_get_temperature() existed.
    """
    getter = getattr(mmwcas, "mmw_get_temperature", None)
    if getter is None:
        return None
    try:
        temps = getter()
    except Exception as e:
        print(f"    [RF TEMP] read failed ({e})")
        return None
    if not temps:
        return None
    for dev, s in sorted(temps.items()):
        print(f"    [RF TEMP] {label:5s} dev{dev}: max {s['max']:3d}C  "
              f"(rx {s['rx0']},{s['rx1']},{s['rx2']},{s['rx3']}  "
              f"tx {s['tx0']},{s['tx1']},{s['tx2']}  "
              f"pm {s['pm']}  dig {s['dig0']})  "
              f"up {s['time_ms'] / 1000.0:.0f}s")
    return temps


def _arm_tda(capture_dir, num_frames, prealloc_files, packing):
    """Arm the TDA, tolerating an mmwcas built before data_packing existed.

    The compiled mmwcas.so goes stale whenever mmwcas.pyx gains a parameter and
    `make build` hasn't been re-run, and the failure lands mid-capture at arm
    time. A 16-bit capture doesn't actually need the new argument, so retry
    without it and carry on; a 12-bit request genuinely can't be honoured by an
    old build, so fail with the fix instead of silently recording 16-bit data
    that downstream would then try to unpack as 12-bit.
    """
    try:
        return mmwcas.mmw_arming_tda(capture_dir, num_frames, prealloc_files,
                                     packing)
    except TypeError:
        if packing:
            raise RuntimeError(
                "--data-packing 1 requires mmwcas to be rebuilt with "
                "data_packing support - run: make build"
            )
        print("    NOTE: installed mmwcas predates the data_packing argument "
              "(run `make build`);")
        print("    continuing at 16-bit.")
        return mmwcas.mmw_arming_tda(capture_dir, num_frames, prealloc_files)


def _payload_sizes(files, cfg):
    """{data_filename: real payload bytes} for each captured device file.

    Payload = frames the TDA indexed (from <dev>_0000_idx.bin) x that device's
    bytes-per-frame. Used to trim pre-allocation padding; files whose frame
    count or per-frame size can't be resolved are omitted, so an unknown never
    turns into a truncation.
    """
    idx_frames = {}
    for name, size in files:
        if not name.endswith("_idx.bin") or not str(size).isdigit():
            continue
        dev_id = _dev_id_from_data_bin(name)
        n = _frames_from_idx_size(size)
        if dev_id is not None and n:
            idx_frames[dev_id] = n

    targets = {}
    for name, size in files:
        if not name.endswith("_data.bin") or not str(size).isdigit():
            continue
        dev_id = _dev_id_from_data_bin(name)
        if dev_id is None or dev_id not in idx_frames:
            continue
        bpf = _expected_bytes_per_frame_for_device(cfg, dev_id)
        if not bpf:
            continue
        payload = idx_frames[dev_id] * bpf
        if 0 < payload < int(size):
            targets[name] = payload
    return targets


def _warn_on_frame_drops(files, num_frames, cfg):
    """
    Cross-check on-disk <dev>_0000_data.bin sizes against the requested frame
    count and print a prominent WARNING if the TDA wrote FEWER frames than
    requested. When frames are missing, point at the cause the evidence
    supports - the TDA capture window opening at arm time while the radar
    starts ~2.1 s later, which TDA_ARM_FRAME_HEADROOM compensates for - and
    refer to parse_idx.py to confirm from the TDA's own per-frame index
    whether the gap is at the tail (truncation), internal (RF skip), or
    scattered (throughput). Duty cycle is printed as context only: presets at
    35% and 98% duty, differing 2x in write rate, both came up exactly
    292/300, which rules out both RF overrun and SSD bandwidth as the
    explanation. Each device file is checked independently (per-device RX
    masks supported). Best-effort: silently returns if the per-frame size
    can't be derived from cfg or no data files are present.

    Returns the number of dropped frames (max across device files, 0 if none).
    """
    data_files = [(name, int(size)) for name, size in files
                  if name.endswith("_data.bin") and str(size).isdigit()]
    if not data_files:
        return 0

    # Frames the TDA indexed, per device. Authoritative: unlike the data file,
    # idx.bin is one entry per captured frame and is never inflated by
    # pre-allocation padding.
    idx_frames = {}
    for name, size in files:
        if not name.endswith("_idx.bin") or not str(size).isdigit():
            continue
        dev_id = _dev_id_from_data_bin(name)
        n = _frames_from_idx_size(size)
        if dev_id is not None and n is not None:
            idx_frames[dev_id] = n

    max_dropped = 0
    lines = []
    any_sized = False
    for name, size in sorted(data_files):
        dev_id = _dev_id_from_data_bin(name)
        bpf = (_expected_bytes_per_frame_for_device(cfg, dev_id)
               if dev_id is not None else None)
        if bpf is None:
            # Fall back: try master mask if name is unrecognised.
            bpf = _expected_bytes_per_frame_for_device(cfg, 0)
        if bpf is None:
            continue
        if bpf == 0:
            lines.append(f"    {name:28s}  RX disabled (expect ~0 bytes, "
                         f"got {size})")
            continue
        any_sized = True
        whole = size // bpf
        partial = (size % bpf) != 0
        # Prefer the index count; fall back to size/bytes-per-frame.
        indexed = idx_frames.get(dev_id)
        counted = indexed if indexed is not None else whole
        dropped = num_frames - counted
        if dropped > 0:
            max_dropped = max(max_dropped, dropped)
        note = ""
        if dropped > 0:
            note = f"   <-- {dropped} dropped ({dropped / num_frames * 100:.1f}%)"
        elif partial and indexed is None:
            note = "   <-- trailing partial frame"
        src = "idx" if indexed is not None else "size"
        detail = ""
        if indexed is not None and indexed != whole:
            # Expected when pre-allocating: the file is padded to a fixed size.
            detail = f" (data file holds {whole})"
        lines.append(f"    {name:28s} {counted:>5d} frame(s) [{src}]"
                     + detail + note)

    if not lines:
        return 0

    print(f"\n{'-'*60}")
    print(f"Frame integrity check  (requested {num_frames} frame(s); "
          f"bytes/frame varies by device RX mask)")
    print(f"{'-'*60}")
    for ln in lines:
        print(ln)
    if 0 < max_dropped <= FRAME_SHORTFALL_NOISE_FLOOR:
        # A frame or two short is the residual tail effect, not the drop bug
        # (which cost 8-20 frames scattered mid-capture). Don't bury it in the
        # full diagnosis - just say so.
        print(f"\n  NOTE: {max_dropped} frame(s) short of {num_frames} - within "
              f"the {FRAME_SHORTFALL_NOISE_FLOOR}-frame")
        print(f"  tail allowance. Run parse_idx.py if you need to confirm there "
              f"are no internal gaps.")
    elif max_dropped > 0:
        print(f"\n  ** WARNING: {max_dropped} frame(s) short of the requested "
              f"{num_frames}. **")
        # A ~3-5% loss is the measured baseline of this rig, not a misconfigured
        # capture, so do not send the reader off tuning parameters. Every
        # plausible knob has been tested and ruled out - see the FRAME DROPS
        # section of this module's docstring for the evidence. Say what is
        # known, and point at the one thing that actually helps: using the
        # per-frame timestamps instead of assuming uniform sampling.
        pct = max_dropped / num_frames * 100.0
        print(f"     Two known mechanisms (see this file's FRAME DROPS "
              f"docstring before retuning):")
        print(f"       ~2 frames  lost at every capture length with no internal "
              f"gap = tail")
        print(f"                  truncation, the whole story for short "
              f"captures;")
        print(f"       the rest   internal single-frame gaps, ~1-2%, same "
              f"indices on every")
        print(f"                  device, cause still unknown.")
        print(f"     Ruled out: page cache/writeback, write rate, SSD saturation, "
              f"RF duty,")
        print(f"     temperature, host wait. Studio at the same 600-frame "
              f"config is clean")
        print(f"     (599/600, zero internal gaps) - so this is a CLI/Ethernet "
              f"arming-path bug,")
        print(f"     not the TDA hardware or the RF schedule.")
        print(f"     Inspect with:  python3 parse_idx.py --fetch <capture_dir>")
        print(f"     IMPORTANT: _idx.bin carries a timestamp per frame. "
              f"Downstream must place")
        print(f"     frames by those timestamps - assuming N contiguous frames "
              f"at dt biases every")
        print(f"     frequency by the drop fraction (~{pct:.0f}% here).")
        active_us = _frame_active_time_us(cfg)
        try:
            period_ms = float(cfg["mimo"]["frame"]["framePeriodicity"])
        except (KeyError, TypeError, ValueError):
            period_ms = None
        if active_us is not None and period_ms and active_us / 1000.0 > period_ms:
            print(f"     NOTE: chirp schedule ({active_us / 1000.0:.1f} ms) "
                  f"EXCEEDS framePeriodicity")
            print(f"     ({period_ms:.0f} ms) - that genuinely cannot be "
                  f"honoured; shorten idleTimes /")
            print(f"     rampEndTime / numLoops or raise framePeriodicity.")
    elif any_sized:
        print(f"\n  OK: all RX-enabled device files contain the full "
              f"{num_frames} frame(s).")
    print(f"{'-'*60}")
    return max_dropped

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

# RF/geometry config dict - selected from radar_config.RADAR_CONFIGS in
# main() based on --radar-config (defaults to cascade_tx3_rx16). Set here so
# run_one_capture() (which reads this module-level global) works. Also read/
# reassigned directly by mimo_interactive.py (DEPRECATED REPL wrapper) so
# that script can reuse this module's capture logic unmodified.
config_dict = get_radar_config(DEFAULT_RADAR_CONFIG)

# IR sensor state - populated by setup_ir_sensor(), consumed by
# run_one_capture(). ir_recording gates whether the callback (which fires on
# every rising edge for as long as GPIO edge detection is armed) actually
# records anything, so edges outside a capture window are discarded, never
# leaking into the wrong experiment's file.
ir_enabled = False
ir_recording = False
ir_timestamps = []


def _ir_sensor_callback(channel):
    if ir_recording:
        ir_timestamps.append(time.time())


def setup_ir_sensor(pin, bounce_ms):
    """
    Arm GPIO rising-edge detection on the IR sensor pin (same BCM pin/pull-up/
    RISING-edge convention as ir_logger.py and receiver_ir.py's TCRT5000
    wiring). Detection stays armed for the whole process lifetime; actual
    timestamp collection is toggled per-capture via the ir_recording flag.
    """
    global ir_enabled
    if not _HAS_GPIO:
        print("[IR] RPi.GPIO not available - IR sensor timestamp logging disabled.")
        return
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=_ir_sensor_callback, bouncetime=bounce_ms)
        ir_enabled = True
        print(f"[IR] Watching GPIO pin {pin} for rising edges (debounce {bounce_ms}ms).")
    except Exception as e:
        print(f"[IR] WARNING: failed to arm GPIO pin {pin} ({e}) - IR sensor timestamp logging disabled.")


def teardown_ir_sensor():
    if ir_enabled and _HAS_GPIO:
        GPIO.cleanup()


def save_ir_timestamps(capture_dir):
    """Write out whatever IR timestamps were collected for this capture's
    framing window and reset the buffer. Saved as a float64 .npy array
    (Unix epoch seconds), same convention as receiver_ir.py's
    gpio_timestamps.npy. Safe to call even if IR logging is disabled
    (no-op)."""
    global ir_timestamps
    if not ir_enabled:
        return None
    os.makedirs("ir_timestamps", exist_ok=True)
    path = os.path.join("ir_timestamps", f"{capture_dir}_ir_timestamps.npy")
    np.save(path, np.array(ir_timestamps, dtype=np.float64))
    print(f"[IR] Saved {len(ir_timestamps)} marker(s) to {path}")
    ir_timestamps = []
    return path


def run_one_capture(exp_label, num_frames, tda_ip, prealloc_files=None,
                    reclaim_padding=True):
    """
    Arm the TDA, record one capture, de-arm, then verify + export the
    .mmwave.json sidecar. Same division of responsibility as mimo.c's
    run_capture(). Safe to call repeatedly after a single mmw_set_config()/
    mmw_init() - reused by the DEPRECATED mimo_interactive.py REPL for
    exactly that, though repeated same-process captures are the case known
    to show SSD/network throughput drift (see mimo.py's module docstring).

    Always resyncs the TDA's system clock to this host's over SSH before
    arming (see utility.sync_tda_clock()) - the TDA has no real time source
    of its own, so without this its OS clock (capture directory file mtimes,
    etc.) drifts from real time. NOTE: this does NOT make the per-frame
    timestamps in <dev>_0000_idx.bin comparable to the IR sensor's host-clock
    timestamps - that field is a separate monotonic counter, unaffected by
    the OS date. Use check_ir_timestamps.py's duration cross-check instead.

    After the capture, also fetches that same idx.bin and exports its
    per-frame timestamps to frame_timestamps/<capture_dir>_frame_timestamps.npy
    (see utility.fetch_frame_timestamps()) - one entry per frame actually
    captured, TDA monotonic clock (seconds, not wall-clock).

    Capture length is num_frames: the host waits
    num_frames × framePeriodicity + one period margin before calling
    mmw_stop_frame(). This is wall-clock-based, not hardware-exact - actual
    byte counts can vary by a few % between runs of the same num_frames due
    to RPC/network timing jitter.

    NOTE: a live mmw_reconfigure_frame_count() (re-issuing MMWL_frameConfig
    mid-session to hard-stop chips at an exact frame count) was tried here
    and reverted - on real hardware it caused mmw_start_frame() to fail
    (some devices left stuck "frame ongoing") on the very first capture of a
    fresh session, which then wedges ALL subsequent frame-config/start calls
    (RL_RET_CODE_FRAME_IS_ONGOING) until the RF chips are power-cycled /
    process restarted. Not worth the reliability cost - see git history for
    the mmwcas.mmw_reconfigure_frame_count() function kept for reference.

    Returns (status, capture_dir) - status is 0 on full success.
    """
    global config_dict
    timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
    capture_dir = f"{exp_label}_{timestamp}"
    period_s = _frame_period_s(config_dict)
    wait_s = _wait_s_for_frames(num_frames, period_s)
    approx_s = num_frames * period_s

    # So .mmwave.json matches this capture even when interactive overrides
    # the default --frames. Does NOT reprogram the RF chips (see docstring) -
    # actual hardware frame count is whatever was configured at mmw_init()
    # time and does not change per-prompt.
    config_dict["mimo"]["frame"]["numFrames"] = num_frames

    print(f"\n>>> Capturing '{capture_dir}' — {num_frames} frames "
          f"(~{approx_s:.1f}s @ {period_s*1000:.0f} ms/frame) ...")

    sync_tda_clock(tda_ip)

    tda_temps_before = _read_tda_temps("pre", tda_ip)

    if prealloc_files is None:
        prealloc_files = _tda_prealloc_files(config_dict, num_frames)
    if prealloc_files:
        print(f"    TDA pre-allocating {prealloc_files} file(s) "
              f"({prealloc_files * TDA_PREALLOC_FILE_BYTES / (1 << 30):.1f} GiB "
              f"reserved per device) to keep file growth off the write path.")
    else:
        print(f"    TDA pre-allocation DISABLED - expect random dropped frames.")

    # Arm with headroom, not num_frames - the TDA window opens now but the
    # radar won't start for ~2.1 s (see TDA_ARM_FRAME_HEADROOM).
    status = _arm_tda(capture_dir, num_frames + TDA_ARM_FRAME_HEADROOM,
                      prealloc_files, _data_packing(config_dict))
    if status != 0:
        print(f"mmw_arming_tda failed (status: {status})")
        return status, capture_dir
    time.sleep(2)

    temps_before = _read_temps("pre")

    # IR recording is only ever "on" for the TDA framing window below - armed
    # right before mmw_start_frame() (not earlier, e.g. before the _read_temps
    # RPC above, whose variable latency would otherwise let in a random,
    # unsynchronized edge or two ahead of frame 0), disarmed when the frame
    # wait ends (before mmw_stop_frame()/de-arm/transfer, which are unrelated
    # to framing and shouldn't pick up stray edges).
    global ir_recording
    if ir_enabled:
        ir_recording = True

    status = mmwcas.mmw_start_frame()
    if status != 0:
        ir_recording = False
        print(f"mmw_start_frame failed (status: {status})")
        return status, capture_dir

    print(f"\n Capturing... ({num_frames} frames, waiting {wait_s:.2f}s)")
    time.sleep(wait_s)
    ir_recording = False
    ir_npy_path = save_ir_timestamps(capture_dir)

    status = mmwcas.mmw_stop_frame()
    if status != 0:
        print(f"mmw_stop_frame failed (status: {status})")
        return status, capture_dir

    temps_after = _read_temps("post")
    if temps_before and temps_after:
        for dev in sorted(set(temps_before) & set(temps_after)):
            rise = temps_after[dev]["max"] - temps_before[dev]["max"]
            print(f"    [RF TEMP] dev{dev} rose {rise:+d}C over this capture "
                  f"({temps_before[dev]['max']}C -> {temps_after[dev]['max']}C)")

    status = mmwcas.mmw_dearming_tda()
    if status != 0:
        print(f"mmw_dearming_tda failed (status: {status})")
        return status, capture_dir

    tda_temps_after = _read_tda_temps("post", tda_ip)
    if tda_temps_before and tda_temps_after:
        hot_b = max(tda_temps_before.values())
        hot_a = max(tda_temps_after.values())
        print(f"    [TDA TEMP] SoC rose {hot_a - hot_b:+.1f}C over this capture "
              f"({hot_b:.1f}C -> {hot_a:.1f}C)")

    # Check if files were actually captured
    print("\n" + "="*60)
    print("Verifying data capture...")
    print("="*60)

    success, file_count, files = check_captured_files(capture_dir, tda_ip)

    if not success:
        print("\n  WARNING: No files found in capture directory!")
        print("\n  Skipping .mmwave.json generation.")
        return status, capture_dir

    # Compare on-disk frame count vs. requested and warn on dropped frames
    # (TDA/SSD couldn't keep up). Does not fail the capture - the data is
    # still usable, just shorter than asked for.
    _warn_on_frame_drops(files, num_frames, config_dict)

    # Per-frame timestamps straight from the TDA's own idx.bin (one entry
    # per frame ACTUALLY captured, so it naturally reflects any drops,
    # unlike assuming N contiguous frames at the nominal period). TDA
    # monotonic clock, not wall-clock - see fetch_frame_timestamps()'s
    # docstring. Saved locally and uploaded alongside the IR timestamps +
    # .mmwave.json below.
    frame_ts_path = fetch_frame_timestamps(
        capture_dir, tda_ip, float(config_dict["mimo"]["frame"]["framePeriodicity"]))

    # Give back the pre-allocation padding (each reserved file is a fixed
    # 2047 MB) so downstream transfer only moves real frames.
    if reclaim_padding:
        truncate_capture_padding(_payload_sizes(files, config_dict),
                                 capture_dir, tda_ip)

    # Generate configuration JSON file only if capture was successful
    json_filename = os.path.join("mmwave_json_files", f"{capture_dir}.mmwave.json")
    print(f"\nGenerating configuration file: {json_filename}")
    export_config_to_json(config_dict, json_filename)

    # Push the IR timestamps + frame timestamps + config sidecar up into the
    # same TDA capture directory as the raw .bin data, so they travel
    # together through whatever downstream transfer step (e.g.
    # fetch_to_usb.sh) pulls the capture off the TDA, instead of needing
    # separate correlation after the fact.
    upload_files_to_tda([ir_npy_path, frame_ts_path, json_filename], capture_dir, tda_ip)

    return status, capture_dir


def _finish_log(tee, capture_dir, capture_ok, tda_ip):
    """Name the log after the capture, close it, and ship it to the TDA.

    Ordering matters: the log is CLOSED before it is uploaded, so the copy on
    the TDA is complete rather than cut off mid-write. The only lines that end
    up in the terminal but not in the file are this function's own, which is the
    right trade - the uploaded log is the artefact that has to be trustworthy.

    Only uploads on a successful capture, since a run that died before the TDA
    directory existed has nowhere to put it; that log stays local under logs/.
    """
    if tee is None:
        return
    if capture_dir:
        tee.rename(os.path.join("logs", f"{capture_dir}.log"))
    path = tee.stop()
    print(f"\n Terminal log saved: {path}")
    if capture_ok and capture_dir:
        upload_files_to_tda([path], capture_dir, tda_ip)


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='TIDEP-01012 MIMO Cascade Radar Control IMRSL')
    parser.add_argument('-d', '--directory',
                        type=str,
                        default='mmwave_python',
                        help='Base directory name for data capture (default: mmwave_python)')
    parser.add_argument('--frames',
                        type=int,
                        default=100,
                        help='Number of radar frames to capture (programs frame.numFrames + '
                             'TDA numberOfFramesToCapture). Default: 100. At 100 ms period, '
                             '100 frames ≈ 10 s, 300 frames ≈ 30 s.')
    parser.add_argument('--tda-ip',
                        type=str,
                        default='192.168.33.180',
                        help='TDA board IP address (default: 192.168.33.180)')
    parser.add_argument('--no-ir',
                        action='store_true',
                        help='Disable IR sensor timestamp logging entirely (default: enabled if '
                             'RPi.GPIO/pin is available).')
    parser.add_argument('--ir-pin',
                        type=int,
                        default=4,
                        help='BCM GPIO pin of the IR sensor (default: 4, matches receiver_ir.py)')
    parser.add_argument('--ir-bounce-ms',
                        type=int,
                        default=200,
                        help='IR sensor software debounce window in ms (default: 200)')
    parser.add_argument('--tda-prealloc-files',
                        type=int,
                        default=-1,
                        help='Files for the TDA to pre-allocate on its SSD before '
                             'recording (numberOfFilesToAllocate). -1 = auto-size '
                             'from the capture (default), 0 = disable. Each file '
                             'reserves a fixed 2047 MB. Without pre-allocation the '
                             'TDA grows the file while frames stream in and drops '
                             'random frames; use 0 only to A/B test that.')
    parser.add_argument('--data-packing',
                        type=int,
                        default=0,
                        choices=(0, 1),
                        help='TDA ADC data packing: 0 = 16-bit as-is (default), '
                             '1 = drop the 4 LSBs and pack 12-bit, which cuts the '
                             'write rate to 75%% and buys SSD headroom. 12-bit '
                             'costs 4 bits of amplitude resolution and REQUIRES '
                             'downstream unpacking - validate phase/displacement '
                             'precision before using it for real captures.')
    parser.add_argument('--no-reclaim-padding',
                        action='store_true',
                        help='Leave pre-allocated capture files at their full '
                             'reserved size instead of truncating them down to the '
                             'frames actually recorded. Only shrinks files, never '
                             'grows them; disable if you want the raw 2047 MB '
                             'files as the TDA wrote them.')
    parser.add_argument('--radar-config',
                        type=str,
                        default=DEFAULT_RADAR_CONFIG,
                        choices=sorted(RADAR_CONFIGS),
                        help='Named RF/geometry preset from radar_configs/*.toml '
                             f"(default: '{DEFAULT_RADAR_CONFIG}'). "
                             'Add new presets as .toml files there, not here.')
    parser.add_argument('--no-log',
                        action='store_true',
                        help='Do not record this run\'s terminal output. By '
                             'default everything printed (including the C-level '
                             'STATUS lines from mmwcas) is written to '
                             'logs/<capture_dir>.log and uploaded into the TDA '
                             'capture directory alongside the .bin data, so the '
                             'frame-drop report and temperatures stay attached '
                             'to the capture they describe.')
    parser.add_argument('--frame-period-ms',
                        type=float,
                        default=None,
                        help='Override the preset framePeriodicity (ms). This is '
                             'the only knob that changes the required WRITE RATE '
                             'while holding frame count and bytes-per-frame '
                             'fixed, so it isolates a producer/consumer rate '
                             'mismatch from a per-frame effect: doubling it '
                             'halves MB/s but records the identical data. '
                             'Diagnostic use - real captures should set the '
                             'period in the preset so .mmwave.json and the '
                             'downstream dt stay consistent.')

    args = parser.parse_args()

    # Register signal handler for Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    # Validate arguments
    if args.frames < 1:
        print("Error: --frames must be >= 1")
        sys.exit(1)
    if args.frame_period_ms is not None and args.frame_period_ms <= 0:
        print("Error: --frame-period-ms must be > 0")
        sys.exit(1)

    # Start logging before ANY radar work, so a run that dies during configure
    # still leaves a log explaining why. The capture directory name doesn't
    # exist yet (its timestamp is minted in run_one_capture), hence the
    # provisional filename and the rename once we know it.
    tee = None
    if not args.no_log:
        provisional = os.path.join(
            "logs", f"mimo_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
        try:
            tee = TeeLogger(provisional).start()
        except (OSError, ImportError) as e:
            print(f"[LOG] terminal logging unavailable ({e}); continuing "
                  f"without it.")
            tee = None

    # Select the RF/geometry preset (see radar_config.py) for this run.
    # deepcopy so CLI --frames can override TOML numFrames without mutating cache.
    global config_dict
    config_dict = copy.deepcopy(get_radar_config(args.radar_config))
    if args.frame_period_ms is not None:
        was = float(config_dict["mimo"]["frame"]["framePeriodicity"])
        config_dict["mimo"]["frame"]["framePeriodicity"] = args.frame_period_ms
        print(f"Frame period     : {args.frame_period_ms:.0f} ms "
              f"(preset {was:.0f} ms) -- OVERRIDDEN, dt differs from the preset")
    period_ms = float(config_dict["mimo"]["frame"]["framePeriodicity"])
    period_s = period_ms / 1000.0
    config_dict["mimo"]["frame"]["numFrames"] = args.frames
    # Packing is a capture-time choice, but every byte-size calculation and the
    # .mmwave.json sidecar has to agree with it, so keep it in the config dict
    # rather than threading it through separately.
    config_dict["mimo"].setdefault("datapath", {})["dataPacking"] = args.data_packing
    approx_s = args.frames * period_s

    print(f"Capture frames   : {args.frames}  (~{approx_s:.1f}s @ {period_ms:.0f} ms/frame)")
    print(f"Radar config     : {args.radar_config}")
    _report_write_budget(config_dict)

    # Configure radar
    status = mmwcas.mmw_set_config(config_dict)
    if status != 0:
        print(f"Configuration error: {status}")
        raise ValueError(f"mmw_set_config failed with status {status}")

    # Initialize radar
    status = mmwcas.mmw_init()
    assert status == 0, ValueError("mmw_init failed")
    time.sleep(2)

    os.makedirs("mmwave_json_files", exist_ok=True)

    if not args.no_ir:
        setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
    else:
        print("[IR] IR sensor timestamp logging disabled (--no-ir).")

    # Single capture, then exit. Repeated back-to-back captures within the
    # same process (formerly --num-loops/--inter-loop-time) were removed:
    # even with a real cooldown sleep between them, capture sizes for
    # identical --frames still drifted smaller run to run - almost certainly
    # an SSD/network throughput ceiling on the TDA (frame drops), not
    # something fixable from the host side. Run mimo.py again (fresh process,
    # fresh mmw_init()) for each capture, or drive repeats externally (a
    # shell loop or cron) so you control the spacing.
    print("\n" + "="*60)
    print(f"Recording frames: {args.frames}  (~{args.frames * period_s:.1f}s)")
    print("="*60)

    capture_dir = None
    capture_ok = False
    try:
        prealloc = (None if args.tda_prealloc_files < 0
                    else args.tda_prealloc_files)
        status, capture_dir = run_one_capture(args.directory, args.frames,
                                             args.tda_ip, prealloc,
                                             not args.no_reclaim_padding)
        if status != 0:
            sys.exit(1)

        print("\n" + "="*60)
        print(f"Data capture {capture_dir} completed successfully!")
        print("="*60)
        capture_ok = True

    except KeyboardInterrupt:
        print("\n\nCapture interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nError during capture: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        teardown_ir_sensor()
        _finish_log(tee, capture_dir, capture_ok, args.tda_ip)

if __name__ == "__main__":
    main()
