"""
frame_sweep.py - measure how the frame-drop rate scales with capture LENGTH.

WHY THIS EXISTS (bug/capture-frame-drop):
Every drop measurement so far was taken at 300 frames, where the loss is ~3-5%.
The known-good mmWave Studio reference capture (Cascade_Configuration_Capture_
calib5s_*.lua) runs nframes_master = 50, i.e. 5 s - so "Studio doesn't drop and
the CLI does" is NOT an apples-to-apples comparison, and length was never a
controlled variable. This sweeps it.

WHAT THE SHAPE OF THE RESULT MEANS - the whole point of the experiment:
  * drop PERCENTAGE flat vs N   -> constant per-frame hazard. A steady-state
                                   loss that scales to any capture length; a
                                   50-frame Studio run would lose 1-2 frames
                                   too and nobody would have noticed.
  * drop COUNT flat vs N        -> fixed per-capture cost (arm/settle/tail), not
                                   a streaming problem. Percentage then falls as
                                   1/N and short captures really are clean.
  * percentage RISING with N    -> genuine cumulative degradation during a
                                   capture (buffer/queue/fill), the only shape
                                   that justifies hunting a build-up mechanism.

EXPERIMENTAL DESIGN - read before changing:
  * Run ORDER IS RANDOMISED across the whole (length x repeat) grid. Running
    25,50,...,300 in ascending order confounds capture length with time-into-
    session; we already mistook a rising drop count for thermal drift once when
    it was just run-to-run scatter. Randomising decorrelates the two.
  * Session elapsed time is logged per run, and the summary regresses drop rate
    against it as well as against N, so a session-drift effect can be SEEN
    rather than assumed absent.
  * REPEATS ARE MANDATORY. Single runs at 300 frames have ranged 280-299
    captured, so one run per length cannot separate any of the three shapes
    above. Default 3; use more if the summary says the spread swamps the trend.
  * Frame counts come from the TDA's own idx ledger (parse_idx._read_entries),
    never from data-file size, which pre-allocation inflates.

USAGE:
    # default sweep: 25/50/100/200/300 frames x 3 repeats, randomised (~15 min)
    python3 frame_sweep.py

    # custom grid
    python3 frame_sweep.py --frames 50,150,300 --repeats 4

    # match the Studio reference arming exactly (no pre-alloc, no arm headroom)
    python3 frame_sweep.py --extra-args "--tda-prealloc-files 0"

    # keep captures on the TDA instead of deleting them after reading the index
    python3 frame_sweep.py --keep

Each capture is deleted from /mnt/ssd after its index is read, so a long sweep
does not fill the TDA SSD (pre-allocation reserves 2 GiB per device per run).
Results are also written to sweep_results/summary.csv for offline plotting.
"""
import argparse
import csv
import os
import random
import re
import shutil
import statistics
import subprocess
import sys
import time

from parse_idx import _read_entries, _median

CAPTURE_RE = re.compile(r"Capturing '([^']+)'")
RESULTS_DIR = "sweep_results"


def _run_capture(label, frames, radar_config, extra_args):
    """Run one mimo.py capture; return (capture_dir, ok). Streams nothing -
    mimo.py is chatty and we only need the directory name and exit status."""
    cmd = [sys.executable, "mimo.py",
           "--radar-config", radar_config,
           "--frames", str(frames),
           "--directory", label]
    if extra_args:
        cmd += extra_args.split()
    res = subprocess.run(cmd, capture_output=True, text=True)
    m = CAPTURE_RE.search(res.stdout)
    capture_dir = m.group(1) if m else None
    if res.returncode != 0 or not capture_dir:
        sys.stderr.write(res.stdout[-2000:] + "\n" + res.stderr[-2000:] + "\n")
    return capture_dir, (res.returncode == 0 and capture_dir is not None)


def _fetch_idx(capture_dir, tda_ip, dest):
    """scp the *_idx.bin for one capture into its own dest dir.

    Own dir per run because parse_idx._scp_fetch reports whatever *_idx.bin is
    sitting in the destination, which would silently re-read the previous run's
    index if they shared a directory.
    """
    os.makedirs(dest, exist_ok=True)
    remote = f"root@{tda_ip}:/mnt/ssd/{capture_dir}/*_idx.bin"
    cmd = ["scp", "-O",
           "-oHostKeyAlgorithms=+ssh-rsa",
           "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
           "-oStrictHostKeyChecking=no",
           "-oConnectTimeout=10",
           remote, dest]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if res.returncode != 0:
        sys.stderr.write(f"  scp idx failed: {res.stderr.strip()}\n")
        return []
    return sorted(os.path.join(dest, f) for f in os.listdir(dest)
                  if f.endswith("_idx.bin"))


def _delete_capture(capture_dir, tda_ip):
    subprocess.run(["ssh",
                    "-oHostKeyAlgorithms=+ssh-rsa",
                    "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
                    "-oStrictHostKeyChecking=no",
                    "-oConnectTimeout=10",
                    f"root@{tda_ip}", f"rm -rf /mnt/ssd/{capture_dir}"],
                   capture_output=True, text=True, timeout=120)


def _gap_positions(entries):
    """Fractional positions (0..1) through the capture of each missed slot.

    Uniform positions => constant hazard per frame. Bunched late => something
    accumulates during the capture. Bunched early => a settling effect.
    Returns (positions, missed_slot_count).
    """
    if len(entries) < 3:
        return [], 0
    ts = [e["timestamp"] for e in entries]
    deltas = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    med = _median(deltas)
    if not med:
        return [], 0
    positions, missed = [], 0
    for i, d in enumerate(deltas):
        if d > med * 1.5:
            slots = round(d / med) - 1
            if slots > 0:
                missed += slots
                positions.append(i / float(len(deltas)))
    return positions, missed


def _analyse_run(idx_paths, requested):
    """Collapse one run's per-device index files into a single result dict."""
    counts, all_pos, missed_total = [], [], 0
    for p in idx_paths:
        try:
            _, entries = _read_entries(p)
        except (OSError, ValueError) as e:
            sys.stderr.write(f"  {p}: {e}\n")
            continue
        counts.append(len(entries))
        pos, missed = _gap_positions(entries)
        all_pos += pos
        missed_total = max(missed_total, missed)
    if not counts:
        return None
    captured = min(counts)
    return {"captured": captured,
            "dropped": requested - captured,
            "devices_agree": len(set(counts)) == 1,
            "gap_positions": all_pos,
            "internal_missed": missed_total}


def _fit_shapes(rows):
    """Fit the three candidate shapes to dropped-vs-length and name the winner.

    THREE models, not two: a constant-count and a constant-rate fit alone cannot
    see acceleration - on synthetic superlinear data (dropped proportional to
    N^2) the rate model wins on residuals and the verdict comes out "RATE-like",
    hiding exactly the cumulative-degradation case we most want to detect. The
    quadratic term is therefore fitted explicitly.

    Each is a one-parameter least-squares fit over the same points, so the
    residual sums are directly comparable.
    """
    ns = [r["frames"] for r in rows]
    ds = [r["dropped"] for r in rows]
    if len(rows) < 3 or not any(ds):
        return "  (not enough data to discriminate)"

    def _through_origin(power):
        """Least-squares coefficient for dropped = c * N**power."""
        basis = [n ** power for n in ns]
        denom = sum(b * b for b in basis)
        c = sum(b * d for b, d in zip(basis, ds)) / denom if denom else 0.0
        return c, sum((d - c * b) ** 2 for b, d in zip(basis, ds))

    b = sum(ds) / len(ds)
    ssr_count = sum((d - b) ** 2 for d in ds)
    rate, ssr_rate = _through_origin(1)
    quad, ssr_quad = _through_origin(2)

    out = [f"  constant-count model : dropped = {b:.1f}"
           f"{'':<12}(residual {ssr_count:>6.0f})",
           f"  constant-rate  model : dropped = {rate*100:.2f}% x N"
           f"{'':<6}(residual {ssr_rate:>6.0f})",
           f"  accelerating   model : dropped = {quad*1e4:.3f}e-4 x N^2"
           f"{'':<1}(residual {ssr_quad:>6.0f})"]

    ranked = sorted((("COUNT", ssr_count), ("RATE", ssr_rate),
                     ("ACCEL", ssr_quad)), key=lambda kv: kv[1])
    best, best_ssr = ranked[0]
    _, second_ssr = ranked[1]
    if best_ssr > second_ssr * 0.7:
        out.append("  -> INCONCLUSIVE: no model clearly wins. Add repeats "
                   "(--repeats) or widen")
        out.append("     the range (--frames) before drawing a conclusion.")
        return "\n".join(out)

    if best == "RATE":
        out.append("  -> RATE-like: loss scales with length, a constant "
                   "per-frame hazard.")
        out.append("     Short captures are not special, they just lose fewer "
                   "frames - so a")
        out.append("     clean 50-frame Studio run would NOT exonerate the CLI.")
    elif best == "COUNT":
        out.append("  -> COUNT-like: a fixed per-capture cost, independent of "
                   "length.")
        out.append("     Look at arm/settle/tail, not at streaming throughput; "
                   "long captures")
        out.append("     are then cheap and the percentage falls as 1/N.")
    else:
        out.append("  -> ACCELERATING: loss grows faster than length. "
                   "Something accumulates")
        out.append("     during a capture (queue/buffer/fill). This is the one "
                   "shape that")
        out.append("     justifies hunting a build-up mechanism, and it caps "
                   "usable duration.")
    return "\n".join(out)


def _corr(xs, ys):
    """Pearson r, or None when undefined (constant input / too few points)."""
    if len(xs) < 3:
        return None
    try:
        return statistics.correlation(xs, ys)
    except (statistics.StatisticsError, AttributeError, ZeroDivisionError):
        return None


def summarise(rows):
    print(f"\n{'='*72}")
    print("PER-RUN RESULTS")
    print(f"{'='*72}")
    print(f"  {'frames':>7} {'rep':>4} {'t+min':>7} {'captured':>9} "
          f"{'dropped':>8} {'drop%':>7} {'internal':>9}  agree")
    for r in rows:
        print(f"  {r['frames']:>7} {r['rep']:>4} {r['elapsed_min']:>7.1f} "
              f"{r['captured']:>9} {r['dropped']:>8} {r['pct']:>6.2f}% "
              f"{r['internal_missed']:>9}  {'yes' if r['devices_agree'] else 'NO'}")

    print(f"\n{'='*72}")
    print("BY CAPTURE LENGTH")
    print(f"{'='*72}")
    print(f"  {'frames':>7} {'runs':>5} {'mean drop':>10} {'sd':>7} "
          f"{'mean drop%':>11} {'range':>12}")
    by_n = {}
    for r in rows:
        by_n.setdefault(r["frames"], []).append(r)
    for n in sorted(by_n):
        rs = by_n[n]
        ds = [x["dropped"] for x in rs]
        ps = [x["pct"] for x in rs]
        sd = statistics.stdev(ds) if len(ds) > 1 else 0.0
        print(f"  {n:>7} {len(rs):>5} {statistics.mean(ds):>10.1f} {sd:>7.1f} "
              f"{statistics.mean(ps):>10.2f}% {min(ds):>5}-{max(ds):<6}")

    print(f"\n{'='*72}")
    print("WHICH SHAPE FITS")
    print(f"{'='*72}")
    print(_fit_shapes(rows))

    # Length vs session-drift: the reason the order was randomised.
    r_len = _corr([r["frames"] for r in rows], [r["pct"] for r in rows])
    r_time = _corr([r["elapsed_min"] for r in rows], [r["pct"] for r in rows])
    print(f"\n  drop% vs capture length : r = "
          + (f"{r_len:+.2f}" if r_len is not None else "n/a"))
    print(f"  drop% vs session elapsed: r = "
          + (f"{r_time:+.2f}" if r_time is not None else "n/a"))
    print("  (order was randomised, so these two are decorrelated - a large")
    print("   |r| on elapsed time would mean the rig drifts regardless of "
          "length.)")

    # Where inside a capture the losses land.
    pos = [p for r in rows for p in r["gap_positions"]]
    if pos:
        thirds = [sum(1 for p in pos if p < 1 / 3),
                  sum(1 for p in pos if 1 / 3 <= p < 2 / 3),
                  sum(1 for p in pos if p >= 2 / 3)]
        print(f"\n  Gap positions within a capture (n={len(pos)}): "
              f"first third {thirds[0]}, middle {thirds[1]}, last {thirds[2]}")
        print("  (roughly equal => constant hazard; rising => something "
              "accumulates mid-capture.)")


def _write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frames", "rep", "elapsed_min", "captured", "dropped",
                    "pct", "internal_missed", "devices_agree", "capture_dir"])
        for r in rows:
            w.writerow([r["frames"], r["rep"], f"{r['elapsed_min']:.2f}",
                        r["captured"], r["dropped"], f"{r['pct']:.3f}",
                        r["internal_missed"], int(r["devices_agree"]),
                        r["capture_dir"]])
    print(f"\nWrote {path}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", default="25,50,100,200,300",
                    help="comma-separated capture lengths (default "
                         "25,50,100,200,300)")
    ap.add_argument("--repeats", type=int, default=3,
                    help="runs per length; >=3 needed to see past the scatter "
                         "(default 3)")
    ap.add_argument("--radar-config", default="cascade_tx3_rx8")
    ap.add_argument("--tda-ip", default="192.168.33.180")
    ap.add_argument("--label", default="sweep")
    ap.add_argument("--extra-args", default="--no-reclaim-padding",
                    help="extra flags passed through to mimo.py (default "
                         "'--no-reclaim-padding' - the capture is deleted "
                         "afterwards, so trimming it first is wasted time)")
    ap.add_argument("--seed", type=int, default=0,
                    help="RNG seed for the run order (fixed so a sweep is "
                         "reproducible)")
    ap.add_argument("--keep", action="store_true",
                    help="keep captures on the TDA (default deletes each one "
                         "after reading its index, to protect SSD space)")
    args = ap.parse_args()

    lengths = [int(x) for x in args.frames.split(",") if x.strip()]
    grid = [(n, rep) for n in lengths for rep in range(1, args.repeats + 1)]
    random.Random(args.seed).shuffle(grid)

    approx_min = sum(n * 0.1 + 45 for n, _ in grid) / 60.0
    print(f"{'='*72}")
    print(f"FRAME-LENGTH SWEEP  -  {len(grid)} runs, randomised order "
          f"(seed {args.seed})")
    print(f"lengths {lengths}, {args.repeats} repeat(s) each, "
          f"config {args.radar_config}")
    print(f"estimated ~{approx_min:.0f} min")
    print(f"{'='*72}")

    if os.path.isdir(RESULTS_DIR):
        shutil.rmtree(RESULTS_DIR)
    os.makedirs(RESULTS_DIR, exist_ok=True)

    t0 = time.time()
    rows = []
    for i, (n, rep) in enumerate(grid, 1):
        elapsed_min = (time.time() - t0) / 60.0
        print(f"\n[{i}/{len(grid)}] {n} frames, repeat {rep} "
              f"(t+{elapsed_min:.1f} min) ...", flush=True)
        capture_dir, ok = _run_capture(f"{args.label}_{n}f_r{rep}", n,
                                       args.radar_config, args.extra_args)
        if not ok:
            print(f"  capture FAILED - skipping this point")
            continue
        idx_paths = _fetch_idx(capture_dir, args.tda_ip,
                               os.path.join(RESULTS_DIR, capture_dir))
        if not args.keep:
            _delete_capture(capture_dir, args.tda_ip)
        if not idx_paths:
            print(f"  no index files retrieved - skipping this point")
            continue
        res = _analyse_run(idx_paths, n)
        if res is None:
            print(f"  index unreadable - skipping this point")
            continue
        res.update(frames=n, rep=rep, elapsed_min=elapsed_min,
                   capture_dir=capture_dir,
                   pct=res["dropped"] / n * 100.0)
        rows.append(res)
        print(f"  captured {res['captured']}/{n}  "
              f"({res['dropped']} dropped, {res['pct']:.2f}%)")

    if not rows:
        print("\nNo successful runs - nothing to summarise.")
        return 1
    summarise(rows)
    _write_csv(rows, os.path.join(RESULTS_DIR, "summary.csv"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
