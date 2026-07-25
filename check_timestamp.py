"""
check_timestamp.py - sanity-check a capture's IR sensor timestamps
(<capture_dir>_ir_timestamps.npy) AND per-frame radar timestamps for
regularity, and cross-check the two against each other.

Two per-frame timestamp sidecars exist (see utility.py):
  - rf_frame_timestamps.npy: the RF/DSP side's own per-frame timestamp,
    decoded from <dev>_0000_idx.bin (see utility.fetch_rf_frame_timestamps()
    / parse_idx.py) - TDA MONOTONIC clock, not wall-clock.
  - rpi_frame_timestamps.npy: this host's estimate of when each of those
    same frames occurred, ON THIS HOST'S OWN CLOCK - built by anchoring the
    RF timestamps' relative spacing to this host's time.time() at the
    moment mmw_start_frame() returned (see utility.save_rpi_frame_timestamps()).
    Directly comparable to the IR sensor timestamps (also host clock).

WHY THIS EXISTS:
mimo.py's IR sensor (setup_ir_sensor()/_ir_sensor_callback()) is mounted on
the same spinning rig as the radar - one rising edge per revolution. At a
constant rev/s, marker-to-marker intervals should be extremely uniform; a
missed trigger (dust, debounce swallowing an edge, disk slip) shows up as
one interval close to 2x (or more) the rest, and a double-trigger (bounce
not fully absorbed by --ir-bounce-ms) shows up as one interval close to 0.
The radar's own per-frame timestamps (one entry per frame ACTUALLY
captured) have the exact same signature for a dropped frame - a gap near
2x the median - so the same regularity check applies to both, just reused
with different labels/units.

IMPORTANT CAVEAT (verified against real hardware 2026-07-25, not assumed):
the per-frame `timestamp` field inside <dev>_0000_idx.bin (see parse_idx.py)
is NOT wall-clock/Unix-epoch time - it is a monotonic counter (observed
~717,218,416 on a TDA that had been up ~12 minutes, i.e. microseconds since
some boot/driver-init reference, nowhere near 1970 or the TDA's own `date`
at capture time). So rf_frame_timestamps.npy is NOT directly comparable to
the IR .npy timestamps (host epoch) or real-world time on its own - an
earlier assumption in this repo's docs was wrong about
utility.sync_tda_clock() fixing that. rpi_frame_timestamps.npy exists
specifically to bridge this gap (see above). Two cross-checks follow from
this:

  * Duration: any frame-timestamp array's span, and the IR timestamps' own
    span (last - first), should roughly agree, since IR recording is gated
    on/off tight around mmw_start_frame()/mmw_stop_frame() (see
    run_one_capture()'s docstring) - both windows are supposed to be
    coextensive with the actual framing window. Valid even against
    rf_frame_timestamps.npy, since duration doesn't need a shared epoch.

  * Nearest-frame time difference: for each IR marker, which captured frame
    is it closest to in time, and by how much? Against rpi_frame_timestamps
    .npy this is a DIRECT, real comparison (same clock, no assumption).
    Against rf_frame_timestamps.npy (or an idx.bin fallback, for captures
    made before rpi_frame_timestamps.npy existed) it instead requires
    aligning the two clocks by assuming the FIRST IR marker and the FIRST
    captured frame happened at approximately the same real moment - only
    approximately true, since IR recording is armed slightly before frame 0
    but the first marker is whenever the rig's next edge happens to occur
    (up to one full rotation period later, not necessarily at t=0).

USAGE:
    # analyse local .npy files (mmwave.json / idx.bin optional but recommended)
    python3 check_timestamp.py capture_ir_timestamps.npy \
        --rpi-frame-ts capture_rpi_frame_timestamps.npy --json capture.mmwave.json

    # fetch the small sidecars for a capture directly from the TDA first
    # (does NOT fetch the multi-GB raw *_data.bin files)
    python3 check_timestamp.py --fetch <capture_dir> --tda-ip 192.168.33.180

    # state the expected rev/s explicitly (else auto-parsed from a
    # "_Nrps" capture directory name if present)
    python3 check_timestamp.py --fetch <capture_dir> --expected-rps 3

Reports, for the IR markers AND (when available) each frame-timestamp
array: count, inter-event interval min/median/max/mean/stdev, measured
rate, every interval more than 1.5x (missed-trigger/dropped-frame
candidate) or less than 0.5x (bounce/double-trigger candidate) the median.
Then: the IR-span-vs-frame-span duration cross-check, and the per-marker
nearest-frame time-difference comparison described above (exact if
rpi_frame_timestamps.npy is available, approximate otherwise).
"""
import argparse
import json
import os
import re
import struct
import subprocess
import sys

import numpy as np

_HEADER_FMT = "<IIIIQ"          # tag, version, flags, numIdx, dataFileSize
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_ENTRY_FMT = "<HHIHH4IIQQ"      # tag,ver,flags,w,h,meta[4],size,timestamp,offset
_ENTRY_SIZE = struct.calcsize(_ENTRY_FMT)


def _scp_fetch_pattern(remote_pattern, tda_ip, dest, timeout=30):
    """Single scp attempt for one remote glob pattern. Returns (ok, stderr)."""
    cmd = [
        "scp", "-O",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}:{remote_pattern}",
        dest,
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "scp timed out"
    return res.returncode == 0, res.stderr.strip()


def fetch_capture_sidecars(capture_dir, tda_ip, dest="."):
    """
    Pull the small per-capture sidecars - IR timestamps .npy, both frame
    timestamp .npy sidecars, .mmwave.json, and every *_idx.bin - for
    capture_dir from /mnt/ssd/<capture_dir>/ on the TDA. Deliberately does
    NOT touch the multi-GB raw *_data.bin files (unlike a full
    capture-directory fetch_to_usb.sh pull).

    Each sidecar type is fetched with its own scp call so a missing one
    (e.g. no idx.bin because --no-ir was used, or an older capture made
    before the frame timestamp sidecars existed) doesn't block fetching the
    others.

    Returns (ir_npy_path, rf_frame_npy_path, rpi_frame_npy_path, json_path,
    idx_path) - any entry is None if not found. idx_path prefers a file
    named "master*" if more than one *_idx.bin landed, matching the "prefer
    master" convention used elsewhere in this repo (e.g. temperature
    reporting); it's a fallback source for RF frame timestamps, for
    captures made before utility.fetch_rf_frame_timestamps() existed.
    """
    remote_base = f"/mnt/ssd/{capture_dir}"
    print(f"Fetching IR/frame/.mmwave.json/idx sidecars for '{capture_dir}' -> {dest}/ ...")
    for pattern, label in [(f"{remote_base}/*_ir_timestamps.npy", "ir_timestamps.npy"),
                            (f"{remote_base}/*_rf_frame_timestamps.npy", "rf_frame_timestamps.npy"),
                            (f"{remote_base}/*_rpi_frame_timestamps.npy", "rpi_frame_timestamps.npy"),
                            (f"{remote_base}/*.mmwave.json", ".mmwave.json"),
                            (f"{remote_base}/*_idx.bin", "*_idx.bin")]:
        ok, err = _scp_fetch_pattern(pattern, tda_ip, dest)
        if not ok:
            print(f"  {label}: fetch failed ({err})")

    # scp drops remote directory structure and lands files flat in dest, so
    # match on the basename even if capture_dir itself has slashes in it
    # (e.g. a manually nested "07-24_3rps/some_capture" path on the TDA).
    base = os.path.basename(capture_dir.rstrip("/"))
    ir_npy_path = os.path.join(dest, f"{base}_ir_timestamps.npy")
    rf_frame_npy_path = os.path.join(dest, f"{base}_rf_frame_timestamps.npy")
    rpi_frame_npy_path = os.path.join(dest, f"{base}_rpi_frame_timestamps.npy")
    json_path = os.path.join(dest, f"{base}.mmwave.json")
    ir_npy_path = ir_npy_path if os.path.exists(ir_npy_path) else None
    rf_frame_npy_path = rf_frame_npy_path if os.path.exists(rf_frame_npy_path) else None
    rpi_frame_npy_path = rpi_frame_npy_path if os.path.exists(rpi_frame_npy_path) else None
    json_path = json_path if os.path.exists(json_path) else None

    idx_candidates = sorted(f for f in os.listdir(dest) if f.endswith("_idx.bin"))
    idx_path = None
    for f in idx_candidates:
        if f.startswith("master"):
            idx_path = os.path.join(dest, f)
            break
    if idx_path is None and idx_candidates:
        idx_path = os.path.join(dest, idx_candidates[0])

    for label, p in [("ir_timestamps.npy", ir_npy_path),
                      ("rf_frame_timestamps.npy", rf_frame_npy_path),
                      ("rpi_frame_timestamps.npy", rpi_frame_npy_path),
                      (".mmwave.json", json_path),
                      ("idx.bin", idx_path)]:
        print(f"  {label:24s}: {p or 'NOT FOUND'}")
    return ir_npy_path, rf_frame_npy_path, rpi_frame_npy_path, json_path, idx_path


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def analyse_intervals(ts, title, unit_name="marker", expected_hz=None):
    """
    Print regularity stats and flag irregular inter-event intervals for a
    monotonic timestamp array. Shared by the IR markers AND either radar
    frame timestamp array - a spinning-rig IR marker and a captured-frame
    index are each supposed to be evenly spaced at their own nominal rate,
    and a gap in either one (missed IR trigger, or a dropped radar frame)
    shows the same signature: one interval well above the local median.

    Returns (deltas, median_interval_s).
    """
    n = len(ts)
    print(f"\n{'='*66}")
    print(title)
    print(f"{'='*66}")
    print(f"  {unit_name} count       : {n}")
    if n < 2:
        print(f"  (need >=2 {unit_name}s for interval analysis)")
        return [], 0.0

    ts_sorted = sorted(float(t) for t in ts)  # should already be monotonic; be defensive
    deltas = [ts_sorted[i + 1] - ts_sorted[i] for i in range(n - 1)]
    med = _median(deltas)
    mean = sum(deltas) / len(deltas)
    var = sum((d - mean) ** 2 for d in deltas) / len(deltas)
    stdev = var ** 0.5
    cv_pct = (stdev / mean * 100.0) if mean else 0.0

    span_s = ts_sorted[-1] - ts_sorted[0]
    print(f"  span               : {span_s:.3f} s (first -> last {unit_name})")
    print(f"  inter-{unit_name} period: min {min(deltas)*1000:.1f}  median "
          f"{med*1000:.1f}  max {max(deltas)*1000:.1f}  ms   "
          f"mean {mean*1000:.1f}  stdev {stdev*1000:.2f} ms  (CV {cv_pct:.2f}%)")
    measured_hz = 1.0 / med if med else 0.0
    print(f"  measured rate      : {measured_hz:.3f} Hz  (1 / median interval)")
    if expected_hz:
        err_pct = (measured_hz - expected_hz) / expected_hz * 100.0
        print(f"  expected rate      : {expected_hz:.3f} Hz   -> measured is "
              f"{err_pct:+.2f}% off")

    # A spinning blade at constant rev/s (or a radar sampling at constant
    # framePeriodicity) should be very uniform, so use the same 1.5x-median
    # heuristic as parse_idx.py's frame-gap hunt for "missed event", plus a
    # <0.5x-median check for "double-count" (IR bounce not fully absorbed
    # by --ir-bounce-ms - not expected for frame timestamps, but harmless to
    # check).
    missed = [(i, d) for i, d in enumerate(deltas) if d > med * 1.5]
    doubled = [(i, d) for i, d in enumerate(deltas) if d < med * 0.5]
    if missed:
        print(f"\n  {len(missed)} interval(s) > 1.5x median (possible missed "
              f"{unit_name} - dust/debounce/dropped frame):")
        for i, d in missed[:40]:
            slots = d / med if med else 0
            print(f"      after #{i:>4d} (t={ts_sorted[i]-ts_sorted[0]:7.3f}s): "
                  f"{d*1000:8.1f} ms = {slots:5.2f}x median")
        if len(missed) > 40:
            print(f"      ... and {len(missed) - 40} more")
    if doubled:
        print(f"\n  {len(doubled)} interval(s) < 0.5x median (possible "
              f"double-count / bounce):")
        for i, d in doubled[:40]:
            print(f"      after #{i:>4d} (t={ts_sorted[i]-ts_sorted[0]:7.3f}s): "
                  f"{d*1000:8.1f} ms")
        if len(doubled) > 40:
            print(f"      ... and {len(doubled) - 40} more")
    if not missed and not doubled:
        print(f"\n  No intervals outside [0.5x, 1.5x] median: {unit_name}s are "
              f"evenly spaced -> no missed or double events detected.")
    return deltas, med


def _read_idx_frame_timestamps(path, period_ms):
    """
    Decode a *_idx.bin's per-frame `timestamp` field into a float64 seconds
    array - same layout/unit-autodetection as utility.fetch_rf_frame_timestamps().
    Fallback source for RF frame timestamps when a capture's
    <capture_dir>_rf_frame_timestamps.npy sidecar isn't available locally
    (e.g. an older capture made before that feature existed).
    """
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < _HEADER_SIZE:
        raise ValueError(f"{path}: too small to hold a header ({len(raw)} B)")
    body = raw[_HEADER_SIZE:]
    n = len(body) // _ENTRY_SIZE
    ts_raw = [struct.unpack_from(_ENTRY_FMT, body, i * _ENTRY_SIZE)[10] for i in range(n)]
    if n < 2:
        return np.array(ts_raw, dtype=np.float64)
    deltas = sorted(ts_raw[i + 1] - ts_raw[i] for i in range(len(ts_raw) - 1))
    med = deltas[len(deltas) // 2]
    to_s = 1e-6 if med >= period_ms * 100 else 1e-3   # us vs ms, like parse_idx.py
    return np.array([t * to_s for t in ts_raw], dtype=np.float64)


def cross_check_duration(ir_ts, frame_ts, frame_label="frame"):
    """Compare the IR marker span against a frame timestamp array's span -
    no clock-alignment assumption needed, just two durations that are
    supposed to roughly agree (see module docstring)."""
    if len(ir_ts) == 0 or len(frame_ts) == 0:
        print(f"\n{'-'*66}")
        print(f"Duration cross-check (IR marker span vs {frame_label} span)")
        print(f"{'-'*66}")
        print(f"  (need at least one IR marker and one frame timestamp - got "
              f"{len(ir_ts)} marker(s), {len(frame_ts)} frame(s) - skipping. "
              f"Zero IR markers usually means the rig wasn't spinning, or "
              f"--ir-bounce-ms/--ir-pin need checking.)")
        return
    ir_span_s = float(max(ir_ts) - min(ir_ts))
    frame_span_s = float(max(frame_ts) - min(frame_ts))
    print(f"\n{'-'*66}")
    print(f"Duration cross-check (IR marker span vs {frame_label} span)")
    print(f"{'-'*66}")
    print(f"  radar frames captured : {len(frame_ts)}  (span {frame_span_s:.3f} s)")
    print(f"  IR marker span         : {ir_span_s:.3f} s ({len(ir_ts)} markers)")
    if frame_span_s > 0:
        diff_pct = (ir_span_s - frame_span_s) / frame_span_s * 100.0
        print(f"  difference              : {ir_span_s - frame_span_s:+.3f} s "
              f"({diff_pct:+.1f}% of the frame window)")
        if abs(diff_pct) > 20:
            print(f"  ** LARGE mismatch - the IR sensor was armed for a "
                  f"noticeably different window than the actual radar "
                  f"framing. Check the rig was actually spinning for the "
                  f"whole capture.")
        else:
            print(f"  -> windows roughly agree; IR recording tracked the "
                  f"framing window as designed (see run_one_capture()'s "
                  f"ir_recording gating).")


def cross_check_nearest(ir_ts, ref_ts, show_n=40, exact=False):
    """
    For each IR marker, find the nearest reference frame timestamp in time
    and report the gap.

    exact=True: ref_ts is already on this host's epoch clock (i.e.
    rpi_frame_timestamps.npy) - a direct, real time-difference comparison,
    no alignment assumption needed.

    exact=False: ref_ts is on the TDA's own monotonic clock (i.e.
    rf_frame_timestamps.npy or an idx.bin fallback) with no shared epoch, so
    this aligns using each series' OWN relative-to-first-sample clock - an
    APPROXIMATION that assumes the first IR marker and first frame happened
    at about the same real moment (see module docstring).
    """
    if len(ir_ts) == 0 or len(ref_ts) == 0:
        print(f"\n(need at least one IR marker and one frame timestamp for the "
              f"nearest-time comparison - got {len(ir_ts)} marker(s), "
              f"{len(ref_ts)} frame(s) - skipping.)")
        return
    ir = np.array(sorted(float(t) for t in ir_ts))
    ref = np.array(sorted(float(t) for t in ref_ts))
    if not exact:
        ir = ir - ir[0]
        ref = ref - ref[0]

    print(f"\n{'-'*66}")
    if exact:
        print("Nearest IR marker <-> RPi-anchored radar frame time differences")
        print(f"{'-'*66}")
        print(f"  Both on this host's clock (rpi_frame_timestamps.npy) - a direct")
        print(f"  time-difference comparison, no alignment assumption needed.")
    else:
        print("Nearest IR marker <-> RF radar frame time differences (APPROXIMATE)")
        print(f"{'-'*66}")
        print(f"  NOTE: aligned by assuming the first IR marker and first frame")
        print(f"  happened at approximately the same moment (see module docstring)")
        print(f"  - an APPROXIMATION, not an exact alignment (could be off by up to")
        print(f"  one IR rotation period). Prefer rpi_frame_timestamps.npy when")
        print(f"  available.")

    insert_idx = np.searchsorted(ref, ir)
    rows = []
    for k, t in enumerate(ir):
        candidates = []
        if insert_idx[k] < len(ref):
            candidates.append(int(insert_idx[k]))
        if insert_idx[k] > 0:
            candidates.append(int(insert_idx[k]) - 1)
        best = min(candidates, key=lambda j: abs(ref[j] - t))
        rows.append((k, t, best, ref[best], t - ref[best]))

    diffs = np.array([abs(r[4]) for r in rows])
    print(f"\n  matched {len(rows)} IR marker(s) to their nearest frame")
    print(f"  |diff| min {diffs.min()*1000:.1f}  median {np.median(diffs)*1000:.1f}  "
          f"max {diffs.max()*1000:.1f}  mean {diffs.mean()*1000:.1f} ms")

    print(f"\n  {'IR#':>5} {'IR t (s)':>12} {'frame#':>7} {'frame t (s)':>14} {'diff (ms)':>10}")
    for k, t, fidx, ft, diff in rows[:show_n]:
        print(f"  {k:>5} {t:>12.3f} {fidx:>7} {ft:>14.3f} {diff*1000:>10.2f}")
    if len(rows) > show_n:
        print(f"  ... and {len(rows) - show_n} more")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npy_file", nargs="?",
                    help="local <capture_dir>_ir_timestamps.npy path")
    ap.add_argument("--fetch", metavar="CAPTURE_DIR",
                    help="scp the IR timestamps, both frame timestamp sidecars, "
                         ".mmwave.json, and idx.bin from /mnt/ssd/<CAPTURE_DIR>/ on "
                         "the TDA first (does not fetch the multi-GB raw *_data.bin)")
    ap.add_argument("--tda-ip", default="192.168.33.180", help="TDA IP for --fetch")
    ap.add_argument("--json", dest="json_file",
                    help=".mmwave.json sidecar, for framePeriodicity_msec "
                         "(overrides whatever --fetch found)")
    ap.add_argument("--rpi-frame-ts", dest="rpi_frame_ts_file",
                    help="local <capture_dir>_rpi_frame_timestamps.npy (host clock, "
                         "directly comparable to IR timestamps - overrides whatever "
                         "--fetch found)")
    ap.add_argument("--rf-frame-ts", dest="rf_frame_ts_file",
                    help="local <capture_dir>_rf_frame_timestamps.npy (TDA monotonic "
                         "clock - overrides whatever --fetch found)")
    ap.add_argument("--idx", dest="idx_file",
                    help="a *_idx.bin sidecar - fallback source for RF frame "
                         "timestamps if no rf_frame_timestamps.npy is found "
                         "(overrides whatever --fetch found)")
    ap.add_argument("--expected-rps", type=float, default=None,
                    help="expected revolutions/sec of the spinning rig, to compare "
                         "against the measured rate. Auto-parsed from a '_Nrps' "
                         "capture directory name if not given.")
    ap.add_argument("--dest", default=".", help="local directory for --fetch (default: .)")
    args = ap.parse_args()

    npy_path = args.npy_file
    rpi_frame_ts_path = args.rpi_frame_ts_file
    rf_frame_ts_path = args.rf_frame_ts_file
    json_path = args.json_file
    idx_path = args.idx_file
    capture_label = args.fetch
    if args.fetch:
        fetched_ir, fetched_rf, fetched_rpi, fetched_json, fetched_idx = \
            fetch_capture_sidecars(args.fetch, args.tda_ip, args.dest)
        npy_path = npy_path or fetched_ir
        rf_frame_ts_path = rf_frame_ts_path or fetched_rf
        rpi_frame_ts_path = rpi_frame_ts_path or fetched_rpi
        json_path = json_path or fetched_json
        idx_path = idx_path or fetched_idx

    if not npy_path:
        ap.error("no IR timestamps .npy (pass a path or use --fetch CAPTURE_DIR)")

    expected_rps = args.expected_rps
    if expected_rps is None:
        m = re.search(r"_(\d+(?:\.\d+)?)rps", capture_label or npy_path)
        if m:
            expected_rps = float(m.group(1))
            print(f"(parsed expected rev/s = {expected_rps} from the capture name)")

    try:
        ir_ts = np.load(npy_path)
    except (OSError, ValueError) as e:
        print(f"ERROR: could not load {npy_path}: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Loaded {len(ir_ts)} IR marker(s) from {npy_path}")

    period_ms = 100.0
    if json_path and os.path.exists(json_path):
        try:
            with open(json_path) as f:
                cfg = json.load(f)
            period_ms = float(cfg["mmWaveDevices"][0]["rfConfig"]
                              ["rlFrameCfg_t"]["framePeriodicity_msec"])
            print(f"framePeriodicity from {json_path}: {period_ms:.0f} ms")
        except (OSError, ValueError, KeyError, IndexError, json.JSONDecodeError) as e:
            print(f"  (could not read framePeriodicity from {json_path}: {e}; "
                  f"using default {period_ms:.0f} ms for idx.bin unit "
                  f"auto-detection)")

    analyse_intervals(ir_ts, "IR sensor timestamps", "marker", expected_rps)

    expected_frame_hz = 1000.0 / period_ms if period_ms else None

    rpi_frame_ts = None
    if rpi_frame_ts_path and os.path.exists(rpi_frame_ts_path):
        try:
            rpi_frame_ts = np.load(rpi_frame_ts_path)
            print(f"\nLoaded {len(rpi_frame_ts)} RPi-anchored frame timestamp(s) "
                  f"from {rpi_frame_ts_path}")
        except (OSError, ValueError) as e:
            print(f"\n  (could not load {rpi_frame_ts_path}: {e})")
    if rpi_frame_ts is not None and len(rpi_frame_ts) >= 2:
        analyse_intervals(rpi_frame_ts, "RPi-anchored radar frame timestamps",
                          "frame", expected_frame_hz)
        cross_check_duration(ir_ts, rpi_frame_ts, "RPi-anchored frame")
        cross_check_nearest(ir_ts, rpi_frame_ts, exact=True)

    rf_frame_ts = None
    if rf_frame_ts_path and os.path.exists(rf_frame_ts_path):
        try:
            rf_frame_ts = np.load(rf_frame_ts_path)
            print(f"\nLoaded {len(rf_frame_ts)} RF frame timestamp(s) from {rf_frame_ts_path}")
        except (OSError, ValueError) as e:
            print(f"\n  (could not load {rf_frame_ts_path}: {e})")
    if rf_frame_ts is None and idx_path:
        try:
            rf_frame_ts = _read_idx_frame_timestamps(idx_path, period_ms)
            print(f"\nDecoded {len(rf_frame_ts)} RF frame timestamp(s) from {idx_path} "
                  f"(no rf_frame_timestamps.npy sidecar found)")
        except (OSError, ValueError) as e:
            print(f"\n  (could not read {idx_path}: {e})")

    if rf_frame_ts is not None and len(rf_frame_ts) >= 2:
        analyse_intervals(rf_frame_ts, "RF radar frame timestamps (TDA monotonic clock)",
                          "frame", expected_frame_hz)
        print(f"\n  NOTE: these are on the TDA's own MONOTONIC clock, not wall-clock")
        print(f"  time (verified empirically), so the following is a DURATION")
        print(f"  comparison and an APPROXIMATE alignment, not exact. Frame drops")
        print(f"  within the window are covered by parse_idx.py, not this.")
        cross_check_duration(ir_ts, rf_frame_ts, "RF frame")
        if rpi_frame_ts is None or len(rpi_frame_ts) < 2:
            cross_check_nearest(ir_ts, rf_frame_ts, exact=False)

    if (rpi_frame_ts is None or len(rpi_frame_ts) < 2) and \
       (rf_frame_ts is None or len(rf_frame_ts) < 2):
        print(f"\n(no frame timestamps available - pass --rpi-frame-ts / "
              f"--rf-frame-ts / --idx, or --fetch a capture that has one - "
              f"skipping frame-vs-IR comparison)")


if __name__ == "__main__":
    main()
