"""
tda_probe.py - watch the TDA's page cache and SSD while it captures, then line
that up against the exact frames it dropped.

WHY THIS EXISTS (bug/capture-frame-drop):
Every hypothesis so far was inferred from the outside - drop counts vs length,
vs data rate, vs temperature - and several looked right and were not. This
observes the machine directly. It samples /proc/meminfo and /proc/diskstats on
the TDA during a capture, reads the dropped-frame times out of the TDA's own
_idx.bin afterwards, and prints both on one timeline.

The two clocks line up for free: _idx.bin timestamps are TDA uptime in
microseconds (observed 3984 s and 4345 s on runs six minutes apart), which is
the same clock as /proc/uptime. No cross-machine time sync is involved.

WHAT IT IS TESTING:
Dropped frames start ~30-40 s into a capture and never at the beginning, and the
TDA has vm.dirty_expire_centisecs = 3000 (30 s) - the age at which the kernel
force-writes dirty pages. If that is the mechanism, the first gap should coincide
with Writeback going non-zero and Dirty falling. If Writeback has been busy from
the first second, or Dirty never climbs, the theory is dead and the timeline says
so immediately. Note the competing evidence: dirty_background_ratio = 10 should
start writeback within seconds at this data rate, well before the 30 s expiry.

The sampler writes to /tmp on the TDA (tmpfs), never to /mnt/ssd, so probing
cannot itself perturb the capture it is measuring.

USAGE:
    python3 tda_probe.py --frames 600 --directory probe_run
    python3 tda_probe.py --frames 600 --directory probe_slow --extra-args "--frame-period-ms 200"

Prints a per-second table of dirty MB, writeback MB, SSD write rate and disk
busy %, with the rows where frames were lost marked, plus a summary comparing
conditions before the first gap against conditions during the gap phase.
"""
import argparse
import os
import subprocess
import sys
import time

from parse_idx import _read_entries, _median

SSH = ["ssh", "-oHostKeyAlgorithms=+ssh-rsa",
       "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
       "-oStrictHostKeyChecking=no", "-oConnectTimeout=10"]
REMOTE_LOG = "/tmp/tda_probe.log"
REMOTE_PID = "/tmp/tda_probe.pid"

# One sample per line: uptime_s dirty_kB writeback_kB sectors_written ms_doing_io
# /proc/diskstats fields for a device row: $10 = sectors written (512 B units),
# $13 = ms spent doing I/O, which gives disk busy time between samples.
SAMPLER = r"""
rm -f {log}
echo $$ > {pid}
while :; do
  u=$(cut -d' ' -f1 /proc/uptime)
  d=$(awk '/^Dirty:/{{print $2}}' /proc/meminfo)
  w=$(awk '/^Writeback:/{{print $2}}' /proc/meminfo)
  s=$(awk '$3=="{dev}"{{print $10, $13}}' /proc/diskstats)
  echo "$u $d $w $s" >> {log}
  sleep {interval}
done
"""


def _ssh(cmd, **kw):
    return subprocess.run(SSH + [f"root@{ARGS.tda_ip}", cmd],
                          capture_output=True, text=True, **kw)


def _start_sampler(interval, dev):
    script = SAMPLER.format(log=REMOTE_LOG, pid=REMOTE_PID, dev=dev,
                            interval=interval)
    proc = subprocess.Popen(SSH + [f"root@{ARGS.tda_ip}", script],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    time.sleep(1.5)                      # let a baseline sample or two land
    return proc


def _stop_sampler(proc):
    """Kill the remote loop by the PID it recorded.

    Killing the local ssh client alone can leave the shell loop running on the
    TDA, where it would keep appending forever and burn CPU on the board we are
    trying to measure - hence the pid file rather than pattern-matching ps
    output, which busybox formats differently anyway.
    """
    _ssh(f"[ -f {REMOTE_PID} ] && kill $(cat {REMOTE_PID}) 2>/dev/null; "
         f"rm -f {REMOTE_PID}; true")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def _fetch_samples():
    res = _ssh(f"cat {REMOTE_LOG}")
    rows = []
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        try:
            rows.append({"t": float(parts[0]), "dirty_kb": int(parts[1]),
                         "wb_kb": int(parts[2]), "sectors": int(parts[3]),
                         "io_ms": int(parts[4])})
        except ValueError:
            continue
    return rows


def _gap_times(capture_dir):
    """Absolute TDA-uptime seconds at which frames went missing, plus the
    capture's start/end, read from whichever device index we can fetch."""
    dest = os.path.join("probe_results", capture_dir)
    os.makedirs(dest, exist_ok=True)
    res = subprocess.run(
        ["scp", "-O", *SSH[1:], f"root@{ARGS.tda_ip}:/mnt/ssd/{capture_dir}/*_idx.bin",
         dest], capture_output=True, text=True, timeout=120)
    if res.returncode != 0:
        print(f"  could not fetch index: {res.stderr.strip()}")
        return [], None, None, 0
    idx = sorted(f for f in os.listdir(dest) if f.endswith("_idx.bin"))
    if not idx:
        return [], None, None, 0
    _, entries = _read_entries(os.path.join(dest, idx[0]))
    if len(entries) < 3:
        return [], None, None, len(entries)
    ts = [e["timestamp"] / 1e6 for e in entries]        # us -> s of TDA uptime
    deltas = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    med = _median(deltas)
    gaps = [ts[i] for i, d in enumerate(deltas) if med and d > med * 1.5]
    return gaps, ts[0], ts[-1], len(entries)


def report(rows, gaps, t_start, t_end, n_frames, requested):
    if len(rows) < 2:
        print("\nNo probe samples retrieved - cannot correlate.")
        return
    print(f"\n{'='*78}")
    print(f"TDA STATE DURING CAPTURE   ({n_frames}/{requested} frames, "
          f"{len(gaps)} gap(s))")
    print(f"{'='*78}")
    print(f"  {'t_cap':>7} {'dirty MB':>9} {'wb MB':>7} {'write MB/s':>11} "
          f"{'disk busy':>10}   frames lost")

    # Condensed: every gap row and its neighbours in full, otherwise one row in
    # five. A 600-frame capture at 200 ms is 120 samples, which buries the few
    # rows that actually matter.
    lost_at = list(gaps)
    marked = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        if t_start is not None and not (t_start - 3 <= b["t"] <= t_end + 3):
            continue
        here = [g for g in lost_at if a["t"] < g <= b["t"]]
        for g in here:
            lost_at.remove(g)
        marked.append({
            "rel": b["t"] - t_start if t_start else b["t"],
            "dirty": b["dirty_kb"] / 1024.0, "wb": b["wb_kb"] / 1024.0,
            "mbps": (b["sectors"] - a["sectors"]) * 512 / 1e6 / dt,
            "busy": (b["io_ms"] - a["io_ms"]) / (dt * 1000.0) * 100.0,
            "lost": len(here)})

    keep = set()
    for i, m in enumerate(marked):
        if m["lost"] or i % 5 == 0 or i < 3 or i >= len(marked) - 3:
            keep.update(range(max(0, i - 1), min(len(marked), i + 2)))
    prev = None
    for i in sorted(keep):
        if prev is not None and i != prev + 1:
            print(f"  {'...':>7}")
        m = marked[i]
        mark = ("  <-- " + "X" * m["lost"]) if m["lost"] else ""
        print(f"  {m['rel']:>7.1f} {m['dirty']:>9.1f} {m['wb']:>7.1f} "
              f"{m['mbps']:>11.1f} {m['busy']:>9.0f}%{mark}")
        prev = i

    if not gaps or t_start is None:
        print("\n  No gaps in this capture - nothing to correlate.")
        return

    first = min(gaps) - t_start
    print(f"\n{'-'*78}")
    print(f"  First frame lost at t+{first:.1f} s of the capture.")

    def _window(lo, hi):
        sel = [r for r in rows
               if lo <= r["t"] - t_start <= hi and r["t"] >= t_start]
        if not sel:
            return None
        return (sum(r["dirty_kb"] for r in sel) / len(sel) / 1024,
                sum(r["wb_kb"] for r in sel) / len(sel) / 1024,
                max(r["wb_kb"] for r in sel) / 1024)

    # The comparison window must be the EARLY capture, not everything up to the
    # first gap. Under the hypothesis we are testing, writeback switches on and
    # frames start dying moments later, so a window ending at the first gap
    # contains the switch-on itself and its peak looks "already active" - which
    # is exactly how a synthetic dirty-expiry dataset got misread as refuting
    # the theory it was built to demonstrate. Use the first half of the pre-gap
    # period, which under the hypothesis is still quiet.
    early_hi = max(first * 0.5, 0.1)
    early = _window(0, early_hi)
    during = _window(first, 1e9)
    if early and during:
        print(f"  {'':26}{'mean dirty':>12} {'mean wb':>10} {'peak wb':>10}")
        print(f"  early capture (0-{early_hi:.0f}s){'':<4}{early[0]:>10.1f}M "
              f"{early[1]:>9.1f}M {early[2]:>9.1f}M")
        print(f"  from the first gap on     {during[0]:>10.1f}M "
              f"{during[1]:>9.1f}M {during[2]:>9.1f}M")
        print()
        if early[2] < 1.0 and during[2] >= 1.0:
            print("  => Writeback was IDLE early and ACTIVE once frames started "
                  "dying.")
            print("     Consistent with dirty-page expiry driving the losses. "
                  "Confirm by lowering")
            print("     vm.dirty_expire_centisecs and checking the ONSET moves "
                  "with it - if the")
            print("     first gap tracks the new expiry time, the mechanism is "
                  "established.")
        elif early[2] >= 1.0:
            print("  => Writeback was ALREADY active early in the capture, long "
                  "before any frame")
            print("     was lost, so delayed writeback is NOT the trigger. "
                  "Rules out dirty_expire.")
        else:
            print("  => Writeback never became significant, so the losses are "
                  "not page-cache")
            print("     driven at all. Look elsewhere - the capture app may "
                  "write with O_DIRECT,")
            print("     in which case none of the vm.dirty_* knobs can matter.")


def main():
    global ARGS
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=600)
    ap.add_argument("--radar-config", default="cascade_tx3_rx8")
    ap.add_argument("--directory", default="probe")
    ap.add_argument("--tda-ip", default="192.168.33.180")
    ap.add_argument("--interval", default="1",
                    help="sampling period in seconds (default 1; busybox sleep "
                         "may not accept fractions)")
    ap.add_argument("--dev", default="nvme0n1",
                    help="block device name in /proc/diskstats")
    ap.add_argument("--extra-args", default="",
                    help="extra flags for mimo.py, e.g. '--frame-period-ms 200'")
    ARGS = ap.parse_args()

    print(f"Starting TDA probe (every {ARGS.interval}s -> {REMOTE_LOG}) ...")
    capture_dir = None
    sampler = _start_sampler(ARGS.interval, ARGS.dev)
    try:
        cmd = [sys.executable, "mimo.py",
               "--radar-config", ARGS.radar_config,
               "--frames", str(ARGS.frames),
               "--directory", ARGS.directory]
        if ARGS.extra_args:
            cmd += ARGS.extra_args.split()
        res = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write(res.stdout)
        for line in res.stdout.splitlines():
            if "Capturing '" in line:
                capture_dir = line.split("Capturing '")[1].split("'")[0]
                break
    finally:
        _stop_sampler(sampler)

    if not capture_dir:
        print("Could not determine the capture directory - aborting.")
        return 1
    rows = _fetch_samples()
    print(f"\nRetrieved {len(rows)} probe sample(s) from the TDA.")
    gaps, t0, t1, n = _gap_times(capture_dir)
    report(rows, gaps, t0, t1, n, ARGS.frames)
    return 0


if __name__ == "__main__":
    sys.exit(main())
