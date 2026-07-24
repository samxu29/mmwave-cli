"""
parse_idx.py - decode a TDA <dev>_0000_idx.bin capture index and locate frame drops.

WHY THIS EXISTS (bug/capture-frame-drop):
The TDA writes one 48-byte index entry per frame it ACTUALLY captured, behind a
24-byte file header. So idx.bin is the TDA's own ground-truth frame ledger,
independent of the raw <dev>_0000_data.bin size. Comparing the two already
proved the shortfall is a REAL drop (idx count == data count == fewer than
requested), not host wall-clock truncation. This tool goes one step further:
it reads each entry's TIMESTAMP and byte OFFSET so we can see *where* in the
capture the frames went missing, which discriminates the cause:

  * gap clustered at the very END (last ts ~= start + N*period, then nothing)
        -> capture stopped early / RF quit emitting near the end (thermal, or
           de-arm cut the tail) - shrinking N, contiguous data.
  * one/few INTERNAL gaps of ~k*period between consecutive captured frames
        -> RF FRAME SKIP: the chip missed k-1 frame triggers mid-run (the tight
           98% duty schedule or PLL/thermal), master+slave skip the same slots.
  * many small IRREGULAR gaps scattered through the run
        -> TDA->SSD / LVDS backpressure hiccups (data-rate bound).

idx.bin layout (little-endian), matches TI cascade post-proc:
    Header (24 B):  uint32 tag, uint32 version, uint32 flags,
                    uint32 numIdx (frames captured), uint64 dataFileSize
    Entry  (48 B):  uint16 tag, uint16 version, uint32 flags,
                    uint16 width, uint16 height, uint32 meta[4],
                    uint32 size, uint64 timestamp, uint64 offset
Timestamp units are the TDA local clock; observed to be microseconds. This tool
auto-detects us vs ms from the median inter-frame delta vs --period-ms and
reports both raw and normalised numbers, so it does not matter if a given SDK
build differs.

USAGE:
    # parse local idx file(s)
    python3 parse_idx.py master_0000_idx.bin slave3_0000_idx.bin

    # fetch both *_idx.bin from a capture dir on the TDA first, then parse
    python3 parse_idx.py --fetch wait_for_frame_260724_152727 --tda-ip 192.168.33.180

    # override the expected frame period / requested count for the drop report
    python3 parse_idx.py master_0000_idx.bin --period-ms 100 --frames 300

Prints, per file: header numIdx vs entries-on-disk, capture span, per-frame
period stats (min/median/max), constant-size check (partial-frame detection),
and an explicit list of every inter-frame gap larger than 1.5x the median
(the candidate skip points) with its position and size in whole periods.
"""
import argparse
import struct
import subprocess
import sys
import os

_HEADER_FMT = "<IIIIQ"          # tag, version, flags, numIdx, dataFileSize
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)          # 24
_ENTRY_FMT = "<HHIHH4IIQQ"      # tag,ver,flags,w,h,meta[4],size,timestamp,offset
_ENTRY_SIZE = struct.calcsize(_ENTRY_FMT)            # 48


def _scp_fetch(capture_dir, tda_ip, dest="."):
    """Pull every *_idx.bin from /mnt/ssd/<capture_dir>/ on the TDA into dest.
    Returns the list of local idx paths fetched (may be empty)."""
    remote = f"root@{tda_ip}:/mnt/ssd/{capture_dir}/*_idx.bin"
    cmd = [
        "scp", "-O",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        remote, dest,
    ]
    print(f"Fetching {remote} -> {dest}/ ...")
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if res.returncode != 0:
        print(f"  scp failed: {res.stderr.strip()}", file=sys.stderr)
        return []
    # scp with a glob doesn't tell us the names; list what landed.
    got = sorted(
        os.path.join(dest, f) for f in os.listdir(dest) if f.endswith("_idx.bin")
    )
    for g in got:
        print(f"  got {g}")
    return got


def _read_entries(path):
    """Return (header_dict, [entry_dict, ...]) for one idx file."""
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < _HEADER_SIZE:
        raise ValueError(f"{path}: too small to hold a header ({len(raw)} B)")
    tag, ver, flags, num_idx, data_size = struct.unpack_from(_HEADER_FMT, raw, 0)
    header = {"tag": tag, "version": ver, "flags": flags,
              "numIdx": num_idx, "dataFileSize": data_size}
    body = raw[_HEADER_SIZE:]
    n_on_disk = len(body) // _ENTRY_SIZE
    if len(body) % _ENTRY_SIZE:
        print(f"  NOTE: {path} body {len(body)} B is not a whole multiple of "
              f"{_ENTRY_SIZE} - trailing {len(body) % _ENTRY_SIZE} B ignored.")
    entries = []
    for i in range(n_on_disk):
        vals = struct.unpack_from(_ENTRY_FMT, body, i * _ENTRY_SIZE)
        # vals: tag,ver,flags,w,h, meta0..3, size, timestamp, offset
        entries.append({"tag": vals[0], "version": vals[1], "flags": vals[2],
                        "width": vals[3], "height": vals[4],
                        "meta": list(vals[5:9]),
                        "size": vals[9], "timestamp": vals[10],
                        "offset": vals[11]})
    return header, entries


def _dump_fields(entries, gap_indices, context=2):
    """Print every raw field for the frames bracketing each gap.

    This is the RF-skip vs TDA-loss discriminator. If one of the meta words (or
    flags) is the RF/CSI2 frame counter, then across a 2-period gap:
      * counter advances by 1  -> the chip never emitted that frame (RF skipped
        the trigger); nothing was lost downstream.
      * counter advances by 2  -> the chip DID emit it and the TDA/write path
        threw it away (capture-side loss).
    Anything constant or equal to a geometry/size value is not a counter.
    """
    show = set()
    for gi in gap_indices:
        for j in range(gi - context, gi + context + 2):
            if 0 <= j < len(entries):
                show.add(j)
    if not show:
        return
    print(f"\n  Raw fields around gaps (looking for a frame counter):")
    print(f"    {'idx':>5} {'flags':>10} {'w':>5} {'h':>5} "
          f"{'meta0':>10} {'meta1':>10} {'meta2':>10} {'meta3':>10} {'size':>10}")
    prev = None
    for j in sorted(show):
        e = entries[j]
        mark = "  <-- gap follows" if j in gap_indices else ""
        if prev is not None and j != prev + 1:
            print(f"    {'...':>5}")
        print(f"    {j:>5} {e['flags']:>10} {e['width']:>5} {e['height']:>5} "
              + " ".join(f"{m:>10}" for m in e["meta"])
              + f" {e['size']:>10}{mark}")
        prev = j


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def analyse(path, period_ms, requested_frames, dump_fields=False):
    header, entries = _read_entries(path)
    n = len(entries)
    print(f"\n{'='*66}")
    print(f"{path}")
    print(f"{'='*66}")
    print(f"  header numIdx      : {header['numIdx']}")
    print(f"  entries on disk    : {n}")
    print(f"  header dataFileSize: {header['dataFileSize']:,} B")
    if header["numIdx"] != n:
        print(f"  ** header/entries mismatch (file truncated mid-write?) **")
    if requested_frames:
        dropped = requested_frames - n
        pct = dropped / requested_frames * 100.0 if requested_frames else 0.0
        tag = f"{dropped} dropped ({pct:.1f}%)" if dropped > 0 else "complete"
        print(f"  requested frames   : {requested_frames}   -> {tag}")

    if n < 2:
        print("  (need >=2 frames for timing analysis)")
        return

    ts = [e["timestamp"] for e in entries]
    deltas = [ts[i + 1] - ts[i] for i in range(n - 1)]
    med = _median(deltas)
    # Auto-detect timestamp units against the expected period.
    if med >= period_ms * 100:          # ~1000x bigger => microseconds
        unit, to_ms = "us", 1e-3
    else:
        unit, to_ms = "ms", 1.0
    span_ms = (ts[-1] - ts[0]) * to_ms
    print(f"  timestamp unit     : {unit} (auto-detected)")
    print(f"  capture span       : {span_ms/1000.0:.3f} s "
          f"({span_ms:.0f} ms across {n} frames)")
    print(f"  inter-frame period : min {min(deltas)*to_ms:.2f}  "
          f"median {med*to_ms:.2f}  max {max(deltas)*to_ms:.2f}  ms "
          f"(nominal {period_ms:.0f})")

    # Constant data-offset stride => every frame same byte size (no partials).
    off = [e["offset"] for e in entries]
    strides = {off[i + 1] - off[i] for i in range(n - 1)}
    if len(strides) == 1:
        print(f"  data stride        : constant {next(iter(strides)):,} B/frame")
    else:
        print(f"  data stride        : VARIES {sorted(strides)} B "
              f"(partial/short frame present)")

    # Where the real payload lives inside the (possibly pre-allocated and
    # therefore padded-to-2047MB) data file. If it starts at 0 and the stride is
    # constant, everything past live_end is padding and can be truncated away
    # before transfer without losing a frame.
    live_end = off[-1] + entries[-1]["size"]
    contiguous = len(strides) == 1 and off[0] == 0
    print(f"  payload extent     : offset {off[0]:,} .. {live_end:,} B"
          + ("  [contiguous from 0]" if contiguous else "  [NOT contiguous from 0]"))
    if contiguous:
        print(f"  -> safe to reclaim padding:  truncate -s {live_end} "
              f"<dev>_0000_data.bin")

    # Gap hunt: any inter-frame delta well above median = missed trigger(s).
    thresh = med * 1.5
    gaps = [(i, deltas[i]) for i in range(len(deltas)) if deltas[i] > thresh]
    total_missing = sum(round(d / med) - 1 for _, d in gaps) if med else 0
    if gaps:
        print(f"\n  {len(gaps)} internal gap(s) > 1.5x median "
              f"(~{total_missing} missed frame slot(s)):")
        for i, d in gaps[:40]:
            slots = d / med if med else 0
            print(f"      after captured frame #{i:>4d} "
                  f"(t={ts[i]*to_ms/1000.0:7.3f}s): gap {d*to_ms:8.2f} ms "
                  f"= {slots:5.2f} periods  -> ~{round(slots)-1} skipped")
        if len(gaps) > 40:
            print(f"      ... and {len(gaps) - 40} more")
        print("  -> internal gaps => RF frame skip / backpressure mid-capture,")
        print("     NOT tail truncation.")
        if dump_fields:
            _dump_fields(entries, [i for i, _ in gaps])
    else:
        print("\n  No internal gaps > 1.5x median: frames captured were evenly")
        print("  spaced. Any shortfall vs requested is a CLEAN EARLY STOP at the")
        print("  tail (RF quit / de-arm cut the end), not mid-run skipping.")
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("idx_files", nargs="*", help="local <dev>_0000_idx.bin path(s)")
    ap.add_argument("--fetch", metavar="CAPTURE_DIR",
                    help="scp every *_idx.bin from /mnt/ssd/<CAPTURE_DIR>/ on the TDA first")
    ap.add_argument("--tda-ip", default="192.168.33.180", help="TDA IP for --fetch")
    ap.add_argument("--period-ms", type=float, default=100.0,
                    help="nominal framePeriodicity in ms (default 100)")
    ap.add_argument("--frames", type=int, default=0,
                    help="requested frame count, for the drop percentage")
    ap.add_argument("--dump-fields", action="store_true",
                    help="dump raw entry fields around each gap, to identify "
                         "whether a frame counter jumps by 1 (RF never emitted "
                         "the frame) or by 2 (TDA lost a frame it received)")
    args = ap.parse_args()

    paths = list(args.idx_files)
    if args.fetch:
        paths = _scp_fetch(args.fetch, args.tda_ip) + paths
    if not paths:
        ap.error("no idx files (pass paths or use --fetch)")

    counts = []
    for p in paths:
        try:
            counts.append(analyse(p, args.period_ms, args.frames,
                                  dump_fields=args.dump_fields))
        except (OSError, ValueError) as e:
            print(f"\n{p}: ERROR {e}", file=sys.stderr)

    real = [c for c in counts if c is not None]
    if len(real) > 1:
        print(f"\n{'-'*66}")
        if len(set(real)) == 1:
            print(f"All {len(real)} device files agree on {real[0]} frames "
                  f"-> synchronised (points to a shared/RF cause, not one bad link).")
        else:
            print(f"Device frame counts DIFFER: {real} -> not fully synchronised "
                  f"(suggests a per-device/link drop, not a shared RF skip).")


if __name__ == "__main__":
    main()
