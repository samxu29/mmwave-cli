"""
check_ir_timestamps.py - sanity-check a capture's IR sensor timestamps
(<capture_dir>_ir_timestamps.npy) for regularity, and cross-check the
recording window's duration against the radar capture's own frame span.

WHY THIS EXISTS:
mimo.py's IR sensor (setup_ir_sensor()/_ir_sensor_callback()) is mounted on
the same spinning rig as the radar - one rising edge per revolution. At a
constant rev/s, marker-to-marker intervals should be extremely uniform; a
missed trigger (dust, debounce swallowing an edge, disk slip) shows up as
one interval close to 2x (or more) the rest, and a double-trigger (bounce
not fully absorbed by --ir-bounce-ms) shows up as one interval close to 0.

IMPORTANT CAVEAT (verified against real hardware 2026-07-25, not assumed):
the per-frame `timestamp` field inside <dev>_0000_idx.bin (see parse_idx.py)
is NOT wall-clock/Unix-epoch time - it is a monotonic counter (observed
~717,218,416 on a TDA that had been up ~12 minutes, i.e. microseconds since
some boot/driver-init reference, nowhere near 1970 or the TDA's own `date`
at capture time). So utility.sync_tda_clock() (which sets the TDA's OS
`date`) does NOT make individual frame timestamps directly comparable to
the IR .npy timestamps (host epoch) - an earlier assumption in this repo's
docs was wrong about that specific mechanism. What CAN be legitimately
cross-checked without a shared epoch is DURATION: idx.bin's frame
timestamps span a length of time, and the IR timestamps span their own
length of time (last - first) - both windows are supposed to be coextensive
with the actual framing window (IR recording is gated on/off tight around
mmw_start_frame()/mmw_stop_frame(), see run_one_capture()'s docstring), so
their DURATIONS should roughly agree even though the two clocks don't share
an origin. That is what the cross-check below actually verifies - not
absolute time alignment.

USAGE:
    # analyse a local .npy (mmwave.json / idx.bin optional but recommended)
    python3 check_ir_timestamps.py capture_ir_timestamps.npy \
        --json capture.mmwave.json --idx master_0000_idx.bin

    # fetch the small sidecars for a capture directly from the TDA first
    # (does NOT fetch the multi-GB raw *_data.bin files)
    python3 check_ir_timestamps.py --fetch <capture_dir> --tda-ip 192.168.33.180

    # state the expected rev/s explicitly (else auto-parsed from a
    # "_Nrps" capture directory name if present)
    python3 check_ir_timestamps.py --fetch <capture_dir> --expected-rps 3

Reports: marker count, inter-marker interval min/median/max/mean/stdev,
measured rev/s, every interval more than 1.5x (missed-trigger candidate) or
less than 0.5x (bounce/double-trigger candidate) the median, and - when an
idx.bin is available - the frame-span vs IR-span duration cross-check
described above.
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
    Pull the small per-capture sidecars - IR timestamps .npy, .mmwave.json,
    and every *_idx.bin - for capture_dir from /mnt/ssd/<capture_dir>/ on
    the TDA. Deliberately does NOT touch the multi-GB raw *_data.bin files
    (unlike a full capture-directory fetch_to_usb.sh pull).

    Each sidecar type is fetched with its own scp call so a missing one
    (e.g. no idx.bin because --no-ir was used, or a capture with RX
    disabled on every device) doesn't block fetching the others.

    Returns (npy_path, json_path, idx_path) - any entry is None if not
    found. idx_path prefers a file named "master*" if more than one
    *_idx.bin landed, matching the "prefer master" convention used
    elsewhere in this repo (e.g. temperature reporting).
    """
    remote_base = f"/mnt/ssd/{capture_dir}"
    print(f"Fetching IR/.mmwave.json/idx sidecars for '{capture_dir}' -> {dest}/ ...")
    for pattern, label in [(f"{remote_base}/*_ir_timestamps.npy", "ir_timestamps.npy"),
                            (f"{remote_base}/*.mmwave.json", ".mmwave.json"),
                            (f"{remote_base}/*_idx.bin", "*_idx.bin")]:
        ok, err = _scp_fetch_pattern(pattern, tda_ip, dest)
        if not ok:
            print(f"  {label}: fetch failed ({err})")

    # scp drops remote directory structure and lands files flat in dest, so
    # match on the basename even if capture_dir itself has slashes in it
    # (e.g. a manually nested "07-24_3rps/some_capture" path on the TDA).
    base = os.path.basename(capture_dir.rstrip("/"))
    npy_path = os.path.join(dest, f"{base}_ir_timestamps.npy")
    json_path = os.path.join(dest, f"{base}.mmwave.json")
    npy_path = npy_path if os.path.exists(npy_path) else None
    json_path = json_path if os.path.exists(json_path) else None

    idx_candidates = sorted(f for f in os.listdir(dest) if f.endswith("_idx.bin"))
    idx_path = None
    for f in idx_candidates:
        if f.startswith("master"):
            idx_path = os.path.join(dest, f)
            break
    if idx_path is None and idx_candidates:
        idx_path = os.path.join(dest, idx_candidates[0])

    for label, p in [("ir_timestamps.npy", npy_path), (".mmwave.json", json_path),
                      ("idx.bin", idx_path)]:
        print(f"  {label:20s}: {p or 'NOT FOUND'}")
    return npy_path, json_path, idx_path


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def analyse_ir_timestamps(ir_ts, expected_rps=None):
    """Print marker regularity stats and flag irregular inter-marker
    intervals. Returns (deltas, median_interval_s)."""
    n = len(ir_ts)
    print(f"\n{'='*66}")
    print("IR sensor timestamps")
    print(f"{'='*66}")
    print(f"  marker count       : {n}")
    if n < 2:
        print("  (need >=2 markers for interval analysis)")
        return [], 0.0

    ts = sorted(float(t) for t in ir_ts)  # should already be monotonic; be defensive
    deltas = [ts[i + 1] - ts[i] for i in range(n - 1)]
    med = _median(deltas)
    mean = sum(deltas) / len(deltas)
    var = sum((d - mean) ** 2 for d in deltas) / len(deltas)
    stdev = var ** 0.5
    cv_pct = (stdev / mean * 100.0) if mean else 0.0

    span_s = ts[-1] - ts[0]
    print(f"  span               : {span_s:.3f} s (first -> last marker)")
    print(f"  inter-marker period: min {min(deltas)*1000:.1f}  median "
          f"{med*1000:.1f}  max {max(deltas)*1000:.1f}  ms   "
          f"mean {mean*1000:.1f}  stdev {stdev*1000:.2f} ms  (CV {cv_pct:.2f}%)")
    measured_rps = 1.0 / med if med else 0.0
    print(f"  measured rev/s     : {measured_rps:.3f}  (1 / median interval)")
    if expected_rps:
        err_pct = (measured_rps - expected_rps) / expected_rps * 100.0
        print(f"  expected rev/s     : {expected_rps:.3f}   -> measured is "
              f"{err_pct:+.2f}% off")

    # A spinning blade at constant rev/s should be very uniform, so use the
    # same 1.5x-median heuristic as parse_idx.py's frame-gap hunt for
    # "missed trigger", plus a <0.5x-median check for "double-trigger"
    # (bounce not fully absorbed by --ir-bounce-ms).
    missed = [(i, d) for i, d in enumerate(deltas) if d > med * 1.5]
    doubled = [(i, d) for i, d in enumerate(deltas) if d < med * 0.5]
    if missed:
        print(f"\n  {len(missed)} interval(s) > 1.5x median (possible missed "
              f"trigger - dust, debounce swallowing an edge, disk slip):")
        for i, d in missed[:40]:
            slots = d / med if med else 0
            print(f"      after marker #{i:>4d} (t={ts[i]-ts[0]:7.3f}s): "
                  f"{d*1000:8.1f} ms = {slots:5.2f}x median")
        if len(missed) > 40:
            print(f"      ... and {len(missed) - 40} more")
    if doubled:
        print(f"\n  {len(doubled)} interval(s) < 0.5x median (possible "
              f"double-trigger / bounce - try raising --ir-bounce-ms):")
        for i, d in doubled[:40]:
            print(f"      after marker #{i:>4d} (t={ts[i]-ts[0]:7.3f}s): "
                  f"{d*1000:8.1f} ms")
        if len(doubled) > 40:
            print(f"      ... and {len(doubled) - 40} more")
    if not missed and not doubled:
        print(f"\n  No intervals outside [0.5x, 1.5x] median: markers are "
              f"evenly spaced -> consistent with a constant-rev/s spin, no "
              f"missed or double triggers detected.")
    return deltas, med


def _read_idx_span_s(path, period_ms):
    """Return (numIdx, span_seconds) from a *_idx.bin's own frame
    timestamps. Unit (us vs ms) is auto-detected the same way as
    parse_idx.py: compare the median inter-frame delta against the nominal
    frame period."""
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < _HEADER_SIZE:
        raise ValueError(f"{path}: too small to hold a header ({len(raw)} B)")
    _, _, _, num_idx, _ = struct.unpack_from(_HEADER_FMT, raw, 0)
    body = raw[_HEADER_SIZE:]
    n = len(body) // _ENTRY_SIZE
    if n < 2:
        return n, 0.0
    ts = [struct.unpack_from(_ENTRY_FMT, body, i * _ENTRY_SIZE)[10] for i in range(n)]
    deltas = [ts[i + 1] - ts[i] for i in range(n - 1)]
    med = _median(deltas)
    to_s = 1e-6 if med >= period_ms * 100 else 1e-3   # us vs ms, like parse_idx.py
    return n, (ts[-1] - ts[0]) * to_s


def cross_check_duration(ir_span_s, ir_marker_count, idx_path, period_ms):
    print(f"\n{'-'*66}")
    print("Duration cross-check vs the radar's own frame capture window")
    print(f"{'-'*66}")
    if not idx_path:
        print("  (no idx.bin available - skipping)")
        return
    try:
        num_idx, frame_span_s = _read_idx_span_s(idx_path, period_ms)
    except (OSError, ValueError) as e:
        print(f"  (could not read {idx_path}: {e})")
        return
    print(f"  radar frames captured : {num_idx}  (frame timestamp span "
          f"{frame_span_s:.3f} s)")
    print(f"  IR marker span         : {ir_span_s:.3f} s "
          f"({ir_marker_count} markers)")
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
    print(f"\n  NOTE: idx.bin's per-frame timestamp is a MONOTONIC counter, not")
    print(f"  wall-clock time (verified empirically - values are on the order")
    print(f"  of TDA uptime in microseconds, not a Unix epoch), so this is a")
    print(f"  DURATION comparison, not an absolute-time alignment check. Frame")
    print(f"  drops within the window are covered by parse_idx.py, not this.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npy_file", nargs="?",
                    help="local <capture_dir>_ir_timestamps.npy path")
    ap.add_argument("--fetch", metavar="CAPTURE_DIR",
                    help="scp the IR timestamps .npy, .mmwave.json, and idx.bin "
                         "sidecars from /mnt/ssd/<CAPTURE_DIR>/ on the TDA first "
                         "(does not fetch the multi-GB raw *_data.bin)")
    ap.add_argument("--tda-ip", default="192.168.33.180", help="TDA IP for --fetch")
    ap.add_argument("--json", dest="json_file",
                    help=".mmwave.json sidecar, for framePeriodicity_msec "
                         "(overrides whatever --fetch found)")
    ap.add_argument("--idx", dest="idx_file",
                    help="a *_idx.bin sidecar, for the duration cross-check "
                         "(overrides whatever --fetch found)")
    ap.add_argument("--expected-rps", type=float, default=None,
                    help="expected revolutions/sec of the spinning rig, to compare "
                         "against the measured rate. Auto-parsed from a '_Nrps' "
                         "capture directory name if not given.")
    ap.add_argument("--dest", default=".", help="local directory for --fetch (default: .)")
    args = ap.parse_args()

    npy_path, json_path, idx_path = args.npy_file, args.json_file, args.idx_file
    capture_label = args.fetch
    if args.fetch:
        fetched_npy, fetched_json, fetched_idx = fetch_capture_sidecars(
            args.fetch, args.tda_ip, args.dest)
        npy_path = npy_path or fetched_npy
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

    deltas, med = analyse_ir_timestamps(ir_ts, expected_rps)
    if deltas:
        ir_span_s = float(max(ir_ts) - min(ir_ts))
        cross_check_duration(ir_span_s, len(ir_ts), idx_path, period_ms)


if __name__ == "__main__":
    main()
