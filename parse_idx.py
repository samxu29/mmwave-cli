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

PER-DEVICE TIME-SLOT MAP (when 2+ idx files are parsed):
Each device already has its own idx.bin. This tool maps every captured
timestamp onto a common slot grid (slot = round((t - t0) / period)) and
reports, per missing slot, WHICH device(s) lack a frame. That is enough to
realign each device's dense data.bin into the correct holes offline.
  * same missing slots on all devices -> shared RF skip (typical tx6 mid-drops)
  * missing slots differ by device     -> per-link / CSI2 / write-path loss

TI onboard cannot add TX-antenna identity or blank hole records into idx.bin
(closed capture path). Cascade chirp sync (master trigger / slave HW sync)
already keeps devices on one frame clock when RF emits; asymmetric drops are
a receive/store issue visible only by comparing the four idx streams.

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
    python3 parse_idx.py rf_bins/master_0000_idx.bin rf_bins/slave3_0000_idx.bin

    # fetch every *_idx.bin from a capture dir on the TDA into ./rf_bins/, then parse
    python3 parse_idx.py --fetch wait_for_frame_260724_152727 --tda-ip 192.168.33.180

    # override the expected frame period / requested count for the drop report
    python3 parse_idx.py rf_bins/master_0000_idx.bin --period-ms 100 --frames 300

    # cross-device slot presence (default when 2+ files) + optional JSON export
    python3 parse_idx.py --fetch <dir> --frames 300 --period-ms 76 \\
        --save-slot-map slot_map.json

Prints, per file: header numIdx vs entries-on-disk, capture span, per-frame
period stats (min/median/max), constant-size check (partial-frame detection),
and an explicit list of every inter-frame gap larger than 1.5x the median
(the candidate skip points) with its position and size in whole periods.
With 2+ devices: a per-slot presence table of who dropped what, plus a
dense-index timestamp-drift check (master_ts[i] vs other_ts[i]). Same file
size / same numIdx does NOT prove content is aligned - if those timestamps
stay locked while data.bin looks skewed, idx cannot name the guilty device.
"""
import argparse
import json
import struct
import subprocess
import sys
import os

_HEADER_FMT = "<IIIIQ"          # tag, version, flags, numIdx, dataFileSize
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)          # 24
_ENTRY_FMT = "<HHIHH4IIQQ"      # tag,ver,flags,w,h,meta[4],size,timestamp,offset
_ENTRY_SIZE = struct.calcsize(_ENTRY_FMT)            # 48

# Basename stem -> cascade device label (RX stream identity, not TX antenna).
_DEV_LABEL = {
    "master": "Dev1/master",
    "slave1": "Dev2/slave1",
    "slave2": "Dev3/slave2",
    "slave3": "Dev4/slave3",
}


def _scp_fetch(capture_dir, tda_ip, dest="rf_bins"):
    """Pull every *_idx.bin from /mnt/ssd/<capture_dir>/ on the TDA into dest.
    Returns the list of local idx paths fetched (may be empty)."""
    os.makedirs(dest, exist_ok=True)
    # Clear prior idx files first. Basenames are always master/slave*_0000_idx.bin
    # (no capture name), so a leftover from another run (or from a capture that
    # dropped a device) will otherwise mix into the parse - and an
    # appear-vs-overwrite check only reports *new* names, missing overwrites.
    for f in os.listdir(dest):
        if f.endswith("_idx.bin"):
            os.remove(os.path.join(dest, f))
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


def _dev_key(path):
    """Short device key from idx basename (master / slave1 / ...)."""
    stem = os.path.basename(path)
    for prefix in ("master", "slave1", "slave2", "slave3"):
        if stem.startswith(prefix):
            return prefix
    return os.path.splitext(stem)[0]


def _dev_label(key):
    return _DEV_LABEL.get(key, key)


def analyse(path, period_ms, requested_frames, dump_fields=False):
    """Print per-file drop analysis. Return a result dict for cross-device map."""
    header, entries = _read_entries(path)
    n = len(entries)
    key = _dev_key(path)
    result = {
        "path": path,
        "key": key,
        "label": _dev_label(key),
        "n": n,
        "ts_ms": [],
        "offsets": [],
        "sizes": [],
        "dense_to_slot": [],
        "period_ms_meas": None,
        "unit": None,
    }
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

    if n < 1:
        print("  (empty idx - no frames)")
        return result
    if n < 2:
        print("  (need >=2 frames for timing analysis)")
        # Still expose the single timestamp for slot mapping.
        # Unit unknown without a delta - assume us (TDA default).
        result["ts_ms"] = [entries[0]["timestamp"] * 1e-3]
        result["offsets"] = [entries[0]["offset"]]
        result["sizes"] = [entries[0]["size"]]
        result["unit"] = "us (assumed; single frame)"
        return result

    ts = [e["timestamp"] for e in entries]
    deltas = [ts[i + 1] - ts[i] for i in range(n - 1)]
    med = _median(deltas)
    # Auto-detect timestamp units against the expected period.
    if med >= period_ms * 100:          # ~1000x bigger => microseconds
        unit, to_ms = "us", 1e-3
    else:
        unit, to_ms = "ms", 1.0
    ts_ms = [t * to_ms for t in ts]
    span_ms = ts_ms[-1] - ts_ms[0]
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
                  f"(t={ts_ms[i]/1000.0:7.3f}s): gap {d*to_ms:8.2f} ms "
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

    result["ts_ms"] = ts_ms
    result["offsets"] = off
    result["sizes"] = [e["size"] for e in entries]
    result["period_ms_meas"] = med * to_ms
    result["unit"] = unit
    return result


def build_slot_map(results, period_ms, requested_frames=0):
    """Map each device's captured frames onto a common integer slot grid.

    slot = round((t_ms - t0_ms) / period_ms), with t0_ms = earliest first-frame
    timestamp across devices. Returns a dict suitable for printing / JSON.
    """
    usable = [r for r in results if r and r.get("ts_ms")]
    if len(usable) < 1:
        return None
    if period_ms <= 0:
        raise ValueError("period_ms must be > 0 for slot mapping")

    t0_ms = min(r["ts_ms"][0] for r in usable)
    # Per device: slot -> dense capture index (first wins if collision).
    by_dev = {}
    all_slots = set()
    for r in usable:
        slot_to_dense = {}
        dense_to_slot = []
        for i, t in enumerate(r["ts_ms"]):
            slot = int(round((t - t0_ms) / period_ms))
            dense_to_slot.append(slot)
            if slot not in slot_to_dense:
                slot_to_dense[slot] = i
            all_slots.add(slot)
        r["dense_to_slot"] = dense_to_slot
        by_dev[r["key"]] = {
            "label": r["label"],
            "path": r["path"],
            "n": r["n"],
            "slot_to_dense": slot_to_dense,
            "dense_to_slot": dense_to_slot,
            "offsets": r["offsets"],
            "sizes": r["sizes"],
        }

    if requested_frames > 0:
        slot_lo, slot_hi = 0, requested_frames - 1
    elif all_slots:
        slot_lo, slot_hi = min(all_slots), max(all_slots)
    else:
        slot_lo, slot_hi = 0, -1

    keys = sorted(by_dev.keys(), key=lambda k: (
        {"master": 0, "slave1": 1, "slave2": 2, "slave3": 3}.get(k, 9), k))
    missing_slots = []
    shared_missing = []
    asymmetric_missing = []
    for s in range(slot_lo, slot_hi + 1):
        present = [s in by_dev[k]["slot_to_dense"] for k in keys]
        if all(present):
            continue
        miss_keys = [k for k, p in zip(keys, present) if not p]
        row = {
            "slot": s,
            "t_ms": t0_ms + s * period_ms,
            "present": {k: bool(p) for k, p in zip(keys, present)},
            "missing": miss_keys,
        }
        missing_slots.append(row)
        if len(miss_keys) == len(keys):
            shared_missing.append(s)
        else:
            asymmetric_missing.append(s)

    return {
        "period_ms": period_ms,
        "t0_ms": t0_ms,
        "slot_lo": slot_lo,
        "slot_hi": slot_hi,
        "n_slots": max(0, slot_hi - slot_lo + 1),
        "devices": keys,
        "device_labels": {k: by_dev[k]["label"] for k in keys},
        "by_dev": by_dev,
        "missing_slots": missing_slots,
        "shared_missing_slots": shared_missing,
        "asymmetric_missing_slots": asymmetric_missing,
    }


def print_slot_map(slot_map, max_rows=80):
    """Print cross-device presence for every incomplete slot."""
    if not slot_map:
        return
    keys = slot_map["devices"]
    labels = [_dev_label(k) for k in keys]
    print(f"\n{'='*66}")
    print("Cross-device time-slot map")
    print(f"{'='*66}")
    print(f"  t0 (earliest first frame) : {slot_map['t0_ms']:.3f} ms (TDA clock)")
    print(f"  period                    : {slot_map['period_ms']:.3f} ms")
    print(f"  slot range                : {slot_map['slot_lo']} .. "
          f"{slot_map['slot_hi']}  ({slot_map['n_slots']} slots)")
    print(f"  devices                   : {', '.join(labels)}")

    n_miss = len(slot_map["missing_slots"])
    n_shared = len(slot_map["shared_missing_slots"])
    n_asym = len(slot_map["asymmetric_missing_slots"])
    if n_miss == 0:
        print("\n  All devices present in every slot of the range "
              "-> no holes to realign.")
        return

    print(f"\n  incomplete slots : {n_miss} "
          f"(shared miss={n_shared}, asymmetric miss={n_asym})")
    if n_shared and not n_asym:
        print("  -> SHARED RF skip pattern: every listed slot is missing on "
              "ALL devices. Realign each device the same way.")
    elif n_asym and not n_shared:
        print("  -> PER-DEVICE / link drops: holes differ by device. "
              "Realign each device from its own dense_to_slot map.")
    elif n_shared and n_asym:
        print("  -> MIXED: some slots missing on all devices, others only on "
              "a subset.")

    # Column header: short keys
    hdr = f"  {'slot':>5} {'t_rel_s':>8}"
    for k in keys:
        hdr += f" {k[:6]:>6}"
    hdr += "  missing"
    print(f"\n{hdr}")
    for i, row in enumerate(slot_map["missing_slots"]):
        if i >= max_rows:
            print(f"  ... and {n_miss - max_rows} more incomplete slots "
                  f"(use --save-slot-map to dump all)")
            break
        t_rel = (row["t_ms"] - slot_map["t0_ms"]) / 1000.0
        line = f"  {row['slot']:>5} {t_rel:8.3f}"
        for k in keys:
            line += f" {'OK':>6}" if row["present"][k] else f" {'DROP':>6}"
        miss_lbl = ",".join(row["missing"])
        line += f"  {miss_lbl}"
        print(line)

    # Per-device summary counts
    print(f"\n  Per-device hole counts in slot range:")
    for k in keys:
        n_drop = sum(1 for row in slot_map["missing_slots"] if k in row["missing"])
        print(f"    {_dev_label(k):16s}  missing {n_drop} slot(s) "
              f"(captured {slot_map['by_dev'][k]['n']})")


def save_slot_map(slot_map, path, drift=None):
    """Write JSON slot map for offline realignment (dense capture idx -> slot)."""
    out = {
        "period_ms": slot_map["period_ms"],
        "t0_ms": slot_map["t0_ms"],
        "slot_lo": slot_map["slot_lo"],
        "slot_hi": slot_map["slot_hi"],
        "n_slots": slot_map["n_slots"],
        "devices": slot_map["devices"],
        "device_labels": slot_map["device_labels"],
        "shared_missing_slots": slot_map["shared_missing_slots"],
        "asymmetric_missing_slots": slot_map["asymmetric_missing_slots"],
        "missing_slots": [
            {
                "slot": r["slot"],
                "t_ms": r["t_ms"],
                "present": r["present"],
                "missing": r["missing"],
            }
            for r in slot_map["missing_slots"]
        ],
        # Post-process: place data.bin frame i into cube[slot] where
        # slot = dense_to_slot[i]; leave holes where present is false.
        "by_dev": {
            k: {
                "label": v["label"],
                "path": v["path"],
                "n_captured": v["n"],
                "dense_to_slot": v["dense_to_slot"],
                "offsets": v["offsets"],
                "sizes": v["sizes"],
            }
            for k, v in slot_map["by_dev"].items()
        },
    }
    if drift is not None:
        out["dense_index_ts_drift"] = drift
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n  Saved slot map -> {path}")


def dense_index_ts_drift(results, period_ms, ref_key="master",
                         jump_frac=0.25, max_jumps=20):
    """Compare timestamps at the same dense index i across devices.

    This is the check for the case where numIdx / file sizes match (so the
    slot map looks 'shared') but data.bin content drifts: if ref_ts[i] and
    other_ts[i] stay locked (~0 diff) while signals desync, idx timestamps
    are not content-truth and cannot name the single-device drop. If the
    dense-index diff jumps by ~1 period at some i, timestamps DO expose skew.
    """
    by_key = {r["key"]: r for r in results if r and r.get("ts_ms")}
    if len(by_key) < 2:
        return None
    if ref_key not in by_key:
        # Fall back to first in cascade order.
        order = ("master", "slave1", "slave2", "slave3")
        ref_key = next((k for k in order if k in by_key), sorted(by_key)[0])
    ref = by_key[ref_key]
    ref_ts = ref["ts_ms"]
    thresh = period_ms * jump_frac
    pairs = []
    others = sorted(
        (k for k in by_key if k != ref_key),
        key=lambda k: {"slave1": 1, "slave2": 2, "slave3": 3}.get(k, 9),
    )
    for k in others:
        other = by_key[k]
        n = min(len(ref_ts), len(other["ts_ms"]))
        diffs = [other["ts_ms"][i] - ref_ts[i] for i in range(n)]
        abs_d = [abs(d) for d in diffs]
        jumps = []
        prev = 0.0
        for i, d in enumerate(diffs):
            # Jump = step change in the dense-index offset (cumulative skew).
            step = d - prev
            if abs(step) >= thresh and i > 0:
                jumps.append({
                    "dense_i": i,
                    "diff_ms": d,
                    "step_ms": step,
                    "approx_periods": step / period_ms if period_ms else 0.0,
                })
            prev = d
        pairs.append({
            "ref": ref_key,
            "other": k,
            "ref_label": ref["label"],
            "other_label": other["label"],
            "n_compared": n,
            "n_ref": len(ref_ts),
            "n_other": len(other["ts_ms"]),
            "diff_min_ms": min(diffs) if diffs else 0.0,
            "diff_max_ms": max(diffs) if diffs else 0.0,
            "diff_median_ms": _median(diffs) if diffs else 0.0,
            "abs_diff_max_ms": max(abs_d) if abs_d else 0.0,
            "jumps": jumps[:max_jumps],
            "n_jumps": len(jumps),
        })
    return {
        "ref": ref_key,
        "period_ms": period_ms,
        "jump_threshold_ms": thresh,
        "pairs": pairs,
    }


def print_dense_index_ts_drift(drift):
    """Print dense-index timestamp lockstep / skew diagnostic."""
    if not drift:
        return
    print(f"\n{'='*66}")
    print("Dense-index timestamp drift  (same FIFO index i across devices)")
    print(f"{'='*66}")
    print(f"  reference device : {_dev_label(drift['ref'])}")
    print(f"  jump threshold   : {drift['jump_threshold_ms']:.2f} ms "
          f"({drift['jump_threshold_ms'] / drift['period_ms']:.2f} x period)")
    print("  Compares other_ts[i] - ref_ts[i]. Same numIdx / same data.bin")
    print("  size is common; this asks whether timestamps still lockstep")
    print("  at each dense i (if yes, idx cannot explain data.bin desync).")

    any_jump = False
    all_locked = True
    for p in drift["pairs"]:
        print(f"\n  {_dev_label(p['ref'])} vs {_dev_label(p['other'])}:")
        print(f"    compared {p['n_compared']} dense indices "
              f"(ref n={p['n_ref']}, other n={p['n_other']})")
        print(f"    diff ms  : min {p['diff_min_ms']:+.3f}  "
              f"median {p['diff_median_ms']:+.3f}  "
              f"max {p['diff_max_ms']:+.3f}  "
              f"|max| {p['abs_diff_max_ms']:.3f}")
        if p["n_ref"] != p["n_other"]:
            all_locked = False
            print(f"    -> counts differ: single-device loss visible as "
                  f"shorter idx (not only content skew).")
        if p["n_jumps"]:
            any_jump = True
            all_locked = False
            print(f"    -> {p['n_jumps']} dense-index offset jump(s) "
                  f">= threshold (timestamps expose skew):")
            for j in p["jumps"]:
                print(f"         at dense i={j['dense_i']:>4d}: "
                      f"step {j['step_ms']:+.2f} ms "
                      f"({j['approx_periods']:+.2f} per)  "
                      f"running diff {j['diff_ms']:+.2f} ms")
            if p["n_jumps"] > len(p["jumps"]):
                print(f"         ... and {p['n_jumps'] - len(p['jumps'])} more")
        elif p["abs_diff_max_ms"] < drift["jump_threshold_ms"]:
            print("    -> LOCKSTEP: timestamps stay matched at every dense i.")
            print("       If data.bin still desyncs across devices, the miss")
            print("       is invisible to idx (no per-payload frame id).")
        else:
            all_locked = False
            print("    -> steady offset (no step jump) - clock skew / t0 "
                  f"bias ~{p['diff_median_ms']:+.3f} ms, not accumulating.")

    print()
    if any_jump:
        print("  Verdict: dense-index timestamp skew IS visible in idx -")
        print("  use jump locations to realign that device vs reference.")
    elif all_locked and all(
            p["n_ref"] == p["n_other"] for p in drift["pairs"]):
        print("  Verdict: all devices timestamp-lockstep at dense i with")
        print("  equal counts. idx cannot identify which device dropped;")
        print("  need a payload frame counter or data.bin correlation.")
    else:
        print("  Verdict: see per-pair notes above.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("idx_files", nargs="*", help="local <dev>_0000_idx.bin path(s)")
    ap.add_argument("--fetch", metavar="CAPTURE_DIR",
                    help="scp every *_idx.bin from /mnt/ssd/<CAPTURE_DIR>/ on the TDA first")
    ap.add_argument("--tda-ip", default="192.168.33.180", help="TDA IP for --fetch")
    ap.add_argument("--dest", default="rf_bins",
                    help="local directory for --fetch (default: rf_bins/)")
    ap.add_argument("--period-ms", type=float, default=100.0,
                    help="nominal framePeriodicity in ms (default 100)")
    ap.add_argument("--frames", type=int, default=0,
                    help="requested frame count, for the drop percentage "
                         "and slot-range upper bound")
    ap.add_argument("--dump-fields", action="store_true",
                    help="dump raw entry fields around each gap, to identify "
                         "whether a frame counter jumps by 1 (RF never emitted "
                         "the frame) or by 2 (TDA lost a frame it received)")
    ap.add_argument("--no-slot-report", action="store_true",
                    help="skip the cross-device time-slot presence table")
    ap.add_argument("--no-ts-drift", action="store_true",
                    help="skip dense-index timestamp drift vs master")
    ap.add_argument("--save-slot-map", metavar="JSON",
                    help="write per-device dense_to_slot + missing-slot table "
                         "(+ drift summary) to JSON for offline realignment")
    args = ap.parse_args()

    paths = list(args.idx_files)
    if args.fetch:
        paths = _scp_fetch(args.fetch, args.tda_ip, args.dest) + paths
    if not paths:
        ap.error("no idx files (pass paths or use --fetch)")

    results = []
    for p in paths:
        try:
            results.append(analyse(p, args.period_ms, args.frames,
                                   dump_fields=args.dump_fields))
        except (OSError, ValueError) as e:
            print(f"\n{p}: ERROR {e}", file=sys.stderr)

    real = [r for r in results if r is not None]
    counts = [r["n"] for r in real]
    if len(counts) > 1:
        print(f"\n{'-'*66}")
        if len(set(counts)) == 1:
            print(f"All {len(counts)} device files agree on {counts[0]} frames "
                  f"-> equal numIdx / typically equal data.bin sizes. That alone "
                  f"does NOT prove content is cross-device aligned; see slot map "
                  f"+ dense-index timestamp drift below.")
        else:
            print(f"Device frame counts DIFFER: {counts} -> not fully "
                  f"synchronised (suggests a per-device/link drop, not a "
                  f"shared RF skip).")

    slot_map = None
    drift = None
    want_slots = (
        (not args.no_slot_report or args.save_slot_map)
        and len(real) >= 1
    )
    if want_slots and not args.no_slot_report and len(real) < 2:
        print(f"\n(slot map needs 2+ idx files to compare devices; "
              f"got {len(real)})")
    elif want_slots and len(real) >= 2:
        try:
            slot_map = build_slot_map(real, args.period_ms, args.frames)
        except ValueError as e:
            print(f"\nslot map ERROR: {e}", file=sys.stderr)
            slot_map = None
        if slot_map and not args.no_slot_report:
            print_slot_map(slot_map)

    if len(real) >= 2 and not args.no_ts_drift:
        drift = dense_index_ts_drift(real, args.period_ms)
        print_dense_index_ts_drift(drift)

    if args.save_slot_map:
        if slot_map is None and len(real) >= 2:
            try:
                slot_map = build_slot_map(real, args.period_ms, args.frames)
            except ValueError as e:
                print(f"\nslot map ERROR: {e}", file=sys.stderr)
        if slot_map:
            save_slot_map(slot_map, args.save_slot_map, drift=drift)
        else:
            print("\n(no slot map to save)", file=sys.stderr)


if __name__ == "__main__":
    main()
