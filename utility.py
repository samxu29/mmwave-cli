import os
import subprocess
import sys
import time


class TeeLogger:
    """Mirror everything the process prints into a log file, live.

    Two design constraints, both learned the hard way:

    1. Redirect at the FILE DESCRIPTOR level, not by replacing sys.stdout. A
       large part of a capture's output - every "STATUS 0 | DEV MAP ..." line,
       the chirp config dump, the TDA arm/framing messages - is printf() from
       the compiled mmwcas extension and never passes through Python's stdout
       object, so a sys.stdout wrapper silently loses exactly the lines that say
       whether the radar configured correctly.

    2. Something OTHER THAN A PYTHON THREAD has to move the bytes. The first
       version used a reader thread and killed live output completely: mmwcas's
       cpdef entry points call into C without releasing the GIL, so during
       mmw_set_config()/mmw_init() - tens of seconds, and where nearly all the
       STATUS lines come from - no Python thread can run, and the terminal sat
       silent until the call returned. Measured with a GIL-holding ctypes call:
       a C line emitted at t+0.00 did not appear until t+3.00. It also risked a
       hard deadlock, since a C write that fills the pty buffer while the GIL is
       held can never be drained. An external `tee` process is immune: the
       kernel and tee move the data with no reference to our interpreter.

    So: fd 1/2 point at a pty slave, and `tee` runs with the pty master as its
    stdin and the real terminal as its stdout. The pty (rather than a plain
    pipe) matters because behind a pipe the C library switches from line to 4 KB
    block buffering, which would batch the C output into bursts; a pty looks
    like a terminal to it, so line buffering and live output are preserved.
    ONLCR is cleared on the slave so the log gets plain \\n, not \\r\\n.
    """

    def __init__(self, path):
        self.path = path
        self._proc = None
        self._saved_out = None
        self._saved_err = None
        self._devnull = None

    def start(self):
        import pty
        import termios

        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        master, slave = pty.openpty()
        try:
            attrs = termios.tcgetattr(slave)
            attrs[1] &= ~termios.ONLCR          # oflag: no \n -> \r\n
            attrs[3] &= ~termios.ECHO           # lflag: nothing echoes back
            termios.tcsetattr(slave, termios.TCSANOW, attrs)
        except termios.error:
            pass                                 # cosmetic only, keep going

        sys.stdout.flush()
        sys.stderr.flush()
        self._saved_out = os.dup(1)
        self._saved_err = os.dup(2)

        # tee's stderr goes to /dev/null on purpose: tearing the pty down at
        # stop() makes its read() fail with EIO, and GNU tee reports that as
        # "read error" on the terminal even though the log is complete.
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        try:
            self._proc = subprocess.Popen(
                ["tee", self.path],
                stdin=master, stdout=self._saved_out, stderr=self._devnull)
        except (OSError, ValueError):
            os.close(master)
            os.close(slave)
            os.close(self._devnull)
            os.close(self._saved_out)
            os.close(self._saved_err)
            self._saved_out = self._saved_err = self._devnull = None
            raise
        os.close(master)                         # tee owns it now

        os.dup2(slave, 1)
        os.dup2(slave, 2)
        os.close(slave)

        # Python's own buffering mode was fixed at interpreter startup from the
        # ORIGINAL stdout, so under `nohup`/`> file` it is block-buffered while
        # the C side stays line-buffered through the pty. The two streams then
        # interleave wrongly and the log reads out of order. Force line
        # buffering so both sides land in the file in the order they happened.
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.reconfigure(line_buffering=True)
            except (AttributeError, ValueError, OSError):
                pass
        return self

    def rename(self, new_path):
        """Move the log to its final name mid-run.

        The capture directory name (with its timestamp) only exists once the
        capture starts, but logging has to be running before that to catch the
        configuration output. So the log opens under a provisional name and is
        renamed here; tee holds an open handle, so the rename follows the data
        and nothing is lost.
        """
        if self._proc is None:
            return self.path
        os.makedirs(os.path.dirname(new_path) or ".", exist_ok=True)
        try:
            os.replace(self.path, new_path)
            self.path = new_path
        except OSError:
            pass
        return self.path

    def stop(self):
        """Restore the real stdout/stderr and let tee finish writing the log."""
        if self._saved_out is None:
            return self.path
        sys.stdout.flush()
        sys.stderr.flush()

        # Restoring fd 1/2 drops the last descriptors on the pty slave, which is
        # what tells tee its input is finished. Waiting for tee to exit is what
        # guarantees the log on disk is complete before anyone uploads it.
        os.dup2(self._saved_out, 1)
        os.dup2(self._saved_err, 2)
        os.close(self._saved_out)
        os.close(self._saved_err)
        self._saved_out = self._saved_err = None
        try:
            self._proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait(timeout=2.0)
        if self._devnull is not None:
            try:
                os.close(self._devnull)
            except OSError:
                pass
            self._devnull = None
        return self.path


def _ls_capture_dir(remote_path, tda_ip):
    """Single SSH `ls` attempt against remote_path. Returns (success, file_count, files)."""
    ssh_cmd = [
        "ssh",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}",
        f"ls -l {remote_path} 2>/dev/null || echo 'DIRECTORY_NOT_FOUND'"
    ]

    result = subprocess.run(
        ssh_cmd,
        capture_output=True,
        text=True,
        timeout=15
    )

    if result.returncode != 0 or "DIRECTORY_NOT_FOUND" in result.stdout:
        return False, 0, []

    output_lines = result.stdout.strip().split('\n')

    # Filter out directory entries (., ..) and get actual files
    files = []
    for line in output_lines:
        if line and not line.startswith('total') and not line.endswith(' .') and not line.endswith(' ..'):
            parts = line.split()
            if len(parts) >= 9:
                # Extract filename (last part) and size
                filename = parts[-1]
                size_str = parts[4]
                files.append((filename, size_str))

    return (len(files) > 0), len(files), files


def check_captured_files(capture_dir, tda_ip="192.168.33.180", retries=4, retry_delay=2.0,
                          settle_checks=4, settle_delay=2.0):
    """
    Check if files were actually captured on the TDA board via SSH, AND that
    they have finished flushing (sizes have stopped changing).

    mmw_dearming_tda() only confirms the TDA firmware accepted the stop
    command - it does NOT wait for buffered frames to actually flush to
    /mnt/ssd/. Immediately after a capture, the directory can transiently
    exist-but-be-empty (or not exist yet) while the flush is still in
    flight, especially for longer captures - the existence-retry loop below
    handles that. But existence alone is not enough: once files *appear*
    they can still be mid-write for a while longer (observed: back-to-back
    captures with identical numFrames produced different final byte counts,
    each smaller than the true frame count, because the next capture's arm/
    reconfigure started while the previous one's data was still landing on
    disk and got cut off). So after files are found, we poll their sizes a
    few more times and only declare success once two consecutive reads are
    byte-identical - i.e. the flush has actually finished, not just started.

    Args:
        capture_dir: The capture directory name
        tda_ip: IP address of the TDA board (default: 192.168.33.180)
        retries: Number of SSH `ls` attempts before giving up on existence (default: 4)
        retry_delay: Seconds to wait between existence-check attempts (default: 2.0)
        settle_checks: Max extra `ls` polls to confirm sizes have stopped growing (default: 4)
        settle_delay: Seconds to wait between settle-check polls (default: 2.0)

    Returns:
        tuple: (success: bool, file_count: int, file_list: list)
    """
    remote_path = f"/mnt/ssd/{capture_dir}"

    print(f"\n{'='*60}")
    print(f"Checking captured files on TDA board...")
    print(f"{'='*60}")
    print(f"Remote path: {remote_path}")

    try:
        for attempt in range(1, retries + 1):
            success, file_count, files = _ls_capture_dir(remote_path, tda_ip)

            if success:
                # Confirm the flush has actually finished: keep polling sizes
                # until two consecutive reads agree, or we run out of budget.
                stable = False
                for settle_attempt in range(1, settle_checks + 1):
                    time.sleep(settle_delay)
                    _, _, files_now = _ls_capture_dir(remote_path, tda_ip)
                    if files_now == files:
                        stable = True
                        break
                    print(f"\n Files still growing (flush in progress, "
                          f"settle check {settle_attempt}/{settle_checks}) - "
                          f"re-checking in {settle_delay}s...")
                    files = files_now
                    file_count = len(files)

                if not stable:
                    print(f"\n WARNING: File sizes did not stabilize after "
                          f"{settle_checks} extra check(s) - reported sizes may "
                          f"still be incomplete. Consider adding --interval / "
                          f"pausing longer between captures.")

                print(f"\n SUCCESS: Found {file_count} file(s) in capture directory"
                      + (f" (attempt {attempt}/{retries})" if attempt > 1 else "")
                      + (" [sizes stable]" if stable else " [sizes NOT confirmed stable]"))
                print(f"\nCaptured files:")
                print(f"{'-'*60}")
                for filename, size in files:
                    print(f"  {filename:50s} {size:>12s}")
                print(f"{'-'*60}")
                return True, file_count, files

            if attempt < retries:
                print(f"\n No files yet (attempt {attempt}/{retries}) - TDA may still be "
                      f"flushing to disk, retrying in {retry_delay}s...")
                time.sleep(retry_delay)

        print(f"\n WARNING: Directory not found or empty after {retries} attempts!")
        print(f"   Path: {remote_path}")
        print(f"   This usually means data capture failed.")
        return False, 0, []

    except subprocess.TimeoutExpired:
        print(f"\n ERROR: SSH connection timeout!")
        print(f"   Could not connect to TDA board at {tda_ip}")
        return False, 0, []
    except Exception as e:
        print(f"\n ERROR: Failed to check files via SSH: {e}")
        return False, 0, []


def read_tda_thermal(tda_ip="192.168.33.180", timeout=15):
    """
    Read the TDA2 SoC thermal zones over SSH. Returns {zone_type: degC} or {}.

    Complements the RF chips' on-die sensors (mmwcas.mmw_get_temperature): those
    only cover the AWR2243 dies, but captured frames go missing from the TDA's
    own index while the RF frame grid stays perfect, so the TDA is at least as
    likely a place for a thermal stall. Measured 70.2 degC on cpu_thermal with
    the board IDLE, which is warm enough to be worth tracking across a capture.

    Zones seen on this board: cpu_thermal, gpu_thermal, core_thermal,
    dspeve_thermal, iva_thermal (sysfs reports millidegrees).

    Never raises - thermal data is diagnostic, so a failed read just yields {}.
    """
    remote = ('for z in /sys/class/thermal/thermal_zone*; do '
              'echo "$(cat $z/type 2>/dev/null) $(cat $z/temp 2>/dev/null)"; done')
    ssh_cmd = [
        "ssh",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}",
        remote,
    ]
    try:
        res = subprocess.run(ssh_cmd, capture_output=True, text=True,
                             timeout=timeout)
        if res.returncode != 0:
            return {}
        zones = {}
        for line in res.stdout.strip().splitlines():
            parts = line.split()
            if len(parts) != 2:
                continue
            name, raw = parts
            try:
                milli = int(raw)
            except ValueError:
                continue
            zones[name.replace("_thermal", "")] = milli / 1000.0
        return zones
    except Exception:
        return {}


def sync_tda_clock(tda_ip="192.168.33.180", timeout=15):
    """
    Set the TDA board's system clock to match this host's wall clock over
    SSH. Returns the offset that was measured before the sync (TDA time
    minus host time, in seconds; None if the read/set failed).

    The TDA sits on an isolated subnet with no route to a real time source,
    so its RTC free-runs from an arbitrary boot value (observed years off,
    not just seconds) while mimo.py's IR sensor timestamps use this host's
    time.time(). Without this, the per-frame timestamps written into
    <dev>_0000_idx.bin (TDA clock) and the IR marker timestamps (host clock)
    are on unrelated time bases and can't be correlated - see "CONSEQUENCE
    FOR DOWNSTREAM" in mimo.py's docstring.

    The TDA's `date` is BusyBox, not GNU coreutils - confirmed by `%N`
    (nanoseconds) coming back as a literal "%N" instead of being expanded.
    Which `-s TIME` syntax this particular build accepts is not something we
    can know in advance (a first guess of the classic POSIX
    "MMDDhhmm[CC]YY.ss" form was rejected as "invalid date" on real
    hardware), so the set step tries a short list of the most common
    `date -s` forms in order and uses whichever one the target accepts,
    rather than betting the whole feature on one guess. Read stays on plain
    `%s` (no `%N`) either way. That caps precision at whole seconds plus one
    SSH round trip (roughly 1-2s), not sub-second - good enough to tell
    which capture session an IR event belongs to, not which exact frame.

    Never raises - a failed sync is a printed warning, not a fatal error,
    since a capture with unsynced clocks is still otherwise usable.
    """
    ssh_base = [
        "ssh",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}",
    ]
    try:
        t0 = time.time()
        res = subprocess.run(ssh_base + ["date +%s"], capture_output=True,
                             text=True, timeout=timeout)
        t1 = time.time()
        if res.returncode != 0:
            print(f"    [CLOCK] WARNING: could not read TDA clock "
                  f"({res.stderr.strip()}) - skipping sync.")
            return None
        tda_before = float(res.stdout.strip())
        offset = tda_before - (t0 + t1) / 2.0

        host_epoch = int(round(time.time()))
        gm = time.gmtime(host_epoch)
        # Try, in order: GNU/newer-BusyBox epoch shorthand; ISO 8601 (BusyBox
        # FEATURE_DATE_ISOFMT, also GNU); classic POSIX positional with a
        # 2-digit year (the oldest, most minimal form); same without a
        # seconds field, in case ".ss" itself is what a given build rejects.
        set_candidates = [
            f"@{host_epoch}",
            time.strftime("%Y-%m-%d %H:%M:%S", gm),
            time.strftime("%m%d%H%M%y.%S", gm),
            time.strftime("%m%d%H%M%y", gm),
        ]
        last_err = ""
        for candidate in set_candidates:
            res2 = subprocess.run(ssh_base + [f"date -u -s '{candidate}' >/dev/null"],
                                  capture_output=True, text=True, timeout=timeout)
            if res2.returncode == 0:
                print(f"    [CLOCK] TDA clock was {offset:+.1f}s vs host - "
                      f"resynced to host time (date -s '{candidate}').")
                return offset
            last_err = res2.stderr.strip()

        print(f"    [CLOCK] TDA clock was {offset:+.1f}s vs host, but no "
              f"supported `date -s` syntax was found (last error: "
              f"{last_err}) - IR/frame timestamps may be misaligned by "
              f"that much.")
        return offset
    except subprocess.TimeoutExpired:
        print(f"    [CLOCK] WARNING: SSH timeout syncing TDA clock - IR/frame "
              f"timestamps may be misaligned.")
        return None
    except Exception as e:
        print(f"    [CLOCK] WARNING: failed to sync TDA clock ({e}).")
        return None


def truncate_capture_padding(targets, capture_dir, tda_ip="192.168.33.180"):
    """
    Shrink pre-allocated capture files on the TDA down to their real payload.

    Arming the TDA with numberOfFilesToAllocate > 0 is what stops it dropping
    frames (it no longer has to extend the file while frames stream in), but TI
    fixes each pre-allocated file at 2047 MB regardless of how many frames were
    actually recorded - so a 300-frame capture writes ~940 MB of data inside a
    2.0 GiB file. Left alone that doubles what downstream transfer has to move.

    Truncating is safe because the frames are written contiguously from offset 0
    with a constant stride (verified via parse_idx.py: "payload extent: offset 0
    .. N [contiguous from 0]", and the idx header's own dataFileSize agrees with
    frames x bytes-per-frame exactly). Everything past the payload is untouched
    pre-allocation, not data.

    targets: {remote_filename: payload_bytes}. Entries are SKIPPED unless the
    file is currently LARGER than payload_bytes, so this can only ever remove
    padding - it will never truncate into real frames, and re-running is a
    no-op. busybox `truncate` is used when present, with a `dd conv=trunc`
    fallback for TDA images that lack it.

    Returns True if the remote command ran, False otherwise (never raises - a
    failed reclaim must not fail an otherwise good capture).
    """
    if not targets:
        return False

    cmds = []
    for name, size in sorted(targets.items()):
        base = str(name).split("/")[-1]
        path = f"/mnt/ssd/{capture_dir}/{base}"
        size = int(size)
        if size <= 0:
            continue
        # Only shrink: compare against the live size on the TDA itself, so a
        # stale/incorrect target can't eat data.
        # stat reads the size from the inode. Do NOT use `wc -c` here: busybox
        # wc streams the whole file to count bytes, so guarding a 2 GiB
        # pre-allocated file that way reads 2 GiB off the SSD per device and
        # times the reclaim out before it truncates anything.
        cmds.append(
            f'cur=$(stat -c %s "{path}" 2>/dev/null || echo 0); '
            f'if [ "$cur" -gt {size} ]; then '
            f'truncate -s {size} "{path}" 2>/dev/null || '
            f'dd if=/dev/null of="{path}" bs=1 seek={size} conv=trunc 2>/dev/null; '
            f'echo "  {base}: $cur -> $(stat -c %s "{path}" 2>/dev/null)"; '
            f'else echo "  {base}: left as-is (size $cur)"; fi'
        )
    if not cmds:
        return False

    ssh_cmd = [
        "ssh",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}",
        "; ".join(cmds),
    ]
    try:
        result = subprocess.run(ssh_cmd, capture_output=True, text=True, timeout=60)
        if result.returncode != 0:
            print(f"\n WARNING: Failed to reclaim pre-allocation padding: "
                  f"{result.stderr.strip()}")
            return False
        out = result.stdout.strip()
        if out:
            print(f"\n Reclaimed pre-allocation padding on the TDA:")
            print(out)
        return True
    except subprocess.TimeoutExpired:
        print(f"\n WARNING: Padding reclaim timed out.")
        return False
    except Exception as e:
        print(f"\n WARNING: Failed to reclaim pre-allocation padding: {e}")
        return False


def upload_files_to_tda(local_paths, capture_dir, tda_ip="192.168.33.180"):
    """
    SCP local metadata files (IR timestamps .npy, .mmwave.json config) up
    into /mnt/ssd/<capture_dir>/ on the TDA board, so they sit alongside the
    raw .bin capture data and travel together through whatever downstream
    transfer step pulls the capture off the TDA, instead of needing a
    separate correlation step downstream.

    local_paths: iterable of local file paths (None/missing entries are
    skipped, e.g. when IR logging was disabled).

    Returns True if the upload succeeded, False otherwise (never raises -
    a failed upload here shouldn't fail the whole capture).
    """
    existing_paths = [p for p in local_paths if p and os.path.exists(p)]
    if not existing_paths:
        return False

    remote_path = f"/mnt/ssd/{capture_dir}/"
    scp_cmd = [
        "scp", "-O",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        *existing_paths,
        f"root@{tda_ip}:{remote_path}",
    ]
    try:
        result = subprocess.run(scp_cmd, capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            print(f"\n WARNING: Failed to upload metadata files to TDA: {result.stderr.strip()}")
            return False
        print(f"\n Uploaded {len(existing_paths)} metadata file(s) to {tda_ip}:{remote_path}"
              f" ({', '.join(os.path.basename(p) for p in existing_paths)})")
        return True
    except subprocess.TimeoutExpired:
        print(f"\n WARNING: SCP upload to TDA timed out.")
        return False
    except Exception as e:
        print(f"\n WARNING: Failed to upload metadata files to TDA: {e}")
        return False


import json
import os
import sys
from datetime import datetime


def _as_hex(value):
    """Format an int (or already-hex str) as Studio-style '0x..' string."""
    if isinstance(value, str):
        return value if value.lower().startswith("0x") else f"0x{int(value, 0):X}"
    return f"0x{int(value):X}"


def _rx_channel_en_for_device(channel, dev_id):
    """Resolve mimo.channel.rxChannelEn for one device (scalar or length-4 list)."""
    rx = channel["rxChannelEn"]
    if isinstance(rx, (list, tuple)):
        return rx[dev_id]
    return rx


def export_config_to_json(config_dict, filename, num_devices=4):
    """
    Create mmwave.json from config_dict (Studio-compatible structure).

    All RF / chirp / frame / datapath fields are taken from config_dict so the
    sidecar matches what was (or will be) programmed - no parallel hardcodes.
    """
    mimo = config_dict["mimo"]
    profile = mimo["profile"]
    frame = mimo["frame"]
    channel = mimo["channel"]
    chirp = mimo["chirp"]
    rf_init = mimo["rfInit"]
    adc_out = mimo["adcOut"]
    low_power = mimo["lowPower"]
    misc = mimo["misc"]
    ldo = mimo["ldo"]
    datapath = mimo["datapath"]

    idle_times = profile["idleTimes"]
    num_profiles = len(idle_times)
    num_chirps = chirp["numChirps"]
    profile_id_per_chirp = chirp["profileIdPerChirp"]
    tx_antenna_table = chirp["txAntennaTable"]

    if len(profile_id_per_chirp) != num_chirps:
        raise ValueError("chirp.profileIdPerChirp length must equal chirp.numChirps")
    if len(tx_antenna_table) < num_devices:
        raise ValueError("chirp.txAntennaTable must have one row per device")
    for row in tx_antenna_table[:num_devices]:
        if len(row) != num_chirps:
            raise ValueError("each chirp.txAntennaTable row must have numChirps entries")

    json_output = {
        "configGenerator": {
            "createdBy": "mmWavel CLI Python Script",
            "createdOn": datetime.now().astimezone().isoformat(),
            "isConfigIntermediate": 1
        },
        "currentVersion": {
            "jsonCfgVersion": {"major": 0, "minor": 4, "patch": 0},
            "DFPVersion": {"major": 2, "minor": 2, "patch": 0},
            "SDKVersion": {"major": 3, "minor": 3, "patch": 0},
            "mmwavelinkVersion": {"major": 2, "minor": 2, "patch": 0}
        },
        "lastBackwardCompatibleVersion": {
            "DFPVersion": {"major": 2, "minor": 1, "patch": 0},
            "SDKVersion": {"major": 3, "minor": 0, "patch": 0},
            "mmwavelinkVersion": {"major": 2, "minor": 1, "patch": 0}
        },
        "regulatoryRestrictions": {
            "frequencyRangeBegin_GHz": 77,
            "frequencyRangeEnd_GHz": 81,
            "maxBandwidthAllowed_MHz": 4000,
            "maxTransmitPowerAllowed_dBm": 12
        },
        "systemConfig": {
            "summary": "This is a comments field not passed to device",
            "sceneParameters": {
                "ambientTemperature_degC": 20,
                "maxDetectableRange_m": 10,
                "rangeResolution_cm": 5,
                "maxVelocity_kmph": 26,
                "velocityResolution_kmph": 2,
                "measurementRate": 10,
                "typicalDetectedObjectRCS": 1.0
            }
        },
        "mmWaveDevices": []
    }

    for devId in range(num_devices):
        chirps = []
        for chirpIdx in range(num_chirps):
            tx_ant = tx_antenna_table[devId][chirpIdx]
            tx_enable = (1 << int(tx_ant)) if tx_ant is not None and int(tx_ant) >= 0 else 0
            chirps.append({
                "rlChirpCfg_t": {
                    "chirpStartIdx": chirpIdx,
                    "chirpEndIdx": chirpIdx,
                    "profileId": profile_id_per_chirp[chirpIdx],
                    "startFreqVar_MHz": float(chirp["startFreqVar_MHz"]),
                    "freqSlopeVar_KHz_usec": float(chirp["freqSlopeVar_KHz_usec"]),
                    "idleTimeVar_usec": float(chirp["idleTimeVar_usec"]),
                    "adcStartTimeVar_usec": float(chirp["adcStartTimeVar_usec"]),
                    "txEnable": _as_hex(tx_enable)
                }
            })

        trigger_select = (
            frame["triggerSelectMaster"] if devId == 0 else frame["triggerSelectSlave"]
        )

        device_config = {
            "mmWaveDeviceId": devId,
            "rfConfig": {
                "waveformType": "legacyFrameChirp",
                "MIMOScheme": "TDM",
                "rlCalibrationDataFile": "",
                "rlChanCfg_t": {
                    "rxChannelEn": _as_hex(_rx_channel_en_for_device(channel, devId)),
                    "txChannelEn": _as_hex(channel["txChannelEn"]),
                    "cascading": 1 if devId == 0 else 2,
                    "cascadingPinoutCfg": "0x0"
                },
                "rlAdcOutCfg_t": {
                    "fmt": {
                        "b2AdcBits": adc_out["adcBits"],
                        "b8FullScaleReducFctr": adc_out["fullScaleReducFctr"],
                        "b2AdcOutFmt": adc_out["adcOutFmt"]
                    }
                },
                "rlLowPowerModeCfg_t": {
                    "lpAdcMode": low_power["lpAdcMode"]
                },
                "rlProfiles": [{
                    "rlProfileCfg_t": {
                        "profileId": pIdx,
                        "pfVcoSelect": _as_hex(profile["pfVcoSelect"]),
                        "pfCalLutUpdate": _as_hex(profile["pfCalLutUpdate"]),
                        "startFreqConst_GHz": profile["startFrequency"],
                        "idleTimeConst_usec": idle_times[pIdx],
                        "adcStartTimeConst_usec": profile["adcStartTime"],
                        "rampEndTime_usec": profile["rampEndTime"],
                        "txOutPowerBackoffCode": _as_hex(profile["txOutPowerBackoffCode"]),
                        "txPhaseShifter": _as_hex(profile["txPhaseShifter"]),
                        "freqSlopeConst_MHz_usec": profile["frequencySlope"],
                        "txStartTime_usec": profile["txStartTime"],
                        "numAdcSamples": profile["numAdcSamples"],
                        "digOutSampleRate": float(profile["adcSamplingFrequency"]),
                        "hpfCornerFreq1": profile["hpfCornerFreq1"],
                        "hpfCornerFreq2": profile["hpfCornerFreq2"],
                        "rxGain_dB": _as_hex(profile["rxGain"])
                    }
                } for pIdx in range(num_profiles)],
                "rlChirps": chirps,
                "rlRfInitCalConf_t": {
                    "calibEnMask": _as_hex(rf_init["calibEnMask"])
                },
                "rlFrameCfg_t": {
                    "chirpEndIdx": frame["chirpEndIdx"],
                    "chirpStartIdx": frame["chirpStartIdx"],
                    "numLoops": frame["numLoops"],
                    "numFrames": frame["numFrames"],
                    "framePeriodicity_msec": float(frame["framePeriodicity"]),
                    "triggerSelect": trigger_select,
                    "frameTriggerDelay": float(frame["frameTriggerDelay"])
                },
                "rlBpmChirps": [],
                "rlRfMiscConf_t": {
                    "miscCtl": str(misc["miscCtl"])
                },
                "rlRfPhaseShiftCfgs": [],
                "rlRfProgFiltConfs": [],
                "rlRfLdoBypassCfg_t": {
                    "ldoBypassEnable": ldo["ldoBypassEnable"],
                    "supplyMonIrDrop": ldo["supplyMonIrDrop"],
                    "ioSupplyIndicator": ldo["ioSupplyIndicator"]
                },
                "rlLoopbackBursts": [],
                "rlDynChirpCfgs": [],
                "rlDynPerChirpPhShftCfgs": []
            },
            "rawDataCaptureConfig": {
                # How the TDA wrote the samples: 0 = 16-bit as-is, 1 = 4 LSBs
                # dropped and packed as 12-bit (read with '*ubit12', not
                # 'uint16'). Recorded per capture so downstream processing can
                # tell without being told out of band.
                "dataPacking": int(datapath.get("dataPacking", 0)),
                "rlDevDataFmtCfg_t": {
                    "iqSwapSel": datapath["iqSwapSel"],
                    "chInterleave": datapath["chInterleave"]
                },
                "rlDevDataPathCfg_t": {
                    "intfSel": datapath["intfSel"],
                    "transferFmtPkt0": _as_hex(datapath["transferFmtPkt0"]),
                    "transferFmtPkt1": _as_hex(datapath["transferFmtPkt1"]),
                    "cqConfig": datapath["cqConfig"],
                    "cq0TransSize": datapath["cq0TransSize"],
                    "cq1TransSize": datapath["cq1TransSize"],
                    "cq2TransSize": datapath["cq2TransSize"]
                },
                "rlDevDataPathClkCfg_t": {
                    "laneClkCfg": datapath["laneClkCfg"],
                    "dataRate_Mbps": datapath["dataRate_Mbps"]
                },
                "rlDevCsi2Cfg_t": {
                    "lanePosPolSel": _as_hex(datapath["lanePosPolSel"]),
                    "lineStartEndDis": datapath["lineStartEndDis"]
                }
            },
            "monitoringConfig": {}
        }
        json_output["mmWaveDevices"].append(device_config)

    json_output["processingChainConfig"] = {
        "detectionChain": {
            "name": "TI_GenericChain",
            "detectionLoss": 1,
            "systemLoss": 1,
            "implementationMargin": 2,
            "detectionSNR": 12,
            "theoreticalRxAntennaGain": 9,
            "theoreticalTxAntennaGain": 9
        }
    }

    try:
        with open(filename, 'w') as f:
            json.dump(json_output, f, indent=2)
        print(f"  > Successfully saved configuration to {filename}")
    except IOError as e:
        print(f"ERROR: Failed to save JSON file: {e}", file=sys.stderr)

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global shutdown_flag
    print("\n\n  Interrupt received (Ctrl+C). Finishing current capture and shutting down...")
    shutdown_flag = True
