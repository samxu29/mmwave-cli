"""
mimo_robotic.py - interactive REPL for mimo.py's radar capture, with a
UR arm motion launched inside distrobox at the moment frames start
(TIDEP-01012 MIMO Cascade Radar, radar mounted on a UR3e TCP).

Each prompt takes an exp_name (same role as mimo.py --exp-name) and an
optional motion command:

    exp_name> test  horz             # radar + arm -> TDA dir horz_test_<YYMMDD>_<HHMMSS>
    exp_name> test                   # radar only, --frames long -> test_<YYMMDD>_<HHMMSS>

With a motion, the motion's short name is prefixed to exp_name, so the TDA
directory (and the trajectory / log files) read <motion>_<exp_name>_<stamp>.
Short names drop the "motion_" prefix (circle, snake_horz, ...); MOTION_ALIASES
shortens a few further (horz = line_horz, vert = line_vert). The long forms
still work at the prompt and give the same short label.

Process layout (same idea as mimo_zed_slave.py + its host):

  * The process you start is a FRONT-END. It owns the prompt and prints a
    compact, coloured summary instead of the full mmwcas/mimo.py report:
    an init progress line, one "recording" line and one result line per take.
    Errors and warnings are still shown in full. Use -v/--verbose for the
    old full output. Every raw line also goes to
    logs/mimo_robotic_session_<YYMMDD_HHMMSS>.log, and each take keeps its
    own logs/<capture_dir>.log (uploaded to the TDA, same as before).

  * The radar itself runs in a WORKER child process (this same file with
    --worker). It does mmw_set_config()/mmw_init() and then mimo.run_one_capture()
    per take, controlled over a JSON line protocol on stdin, and reports
    back with "@MRB {json}" event lines.

  * Re-flash on failure. mmw_init() is not safe to repeat inside one process,
    and mmwcas exits the process outright on a required init step failing
    (e.g. "STATUS -8 ... Failed to enable master RF"). So:
      - init failure: the worker dies, the front-end starts a new one, which
        powers up the chips and downloads the firmware again. Retried
        --init-retries times (default 3); after that it asks you to
        power-cycle the board and press Enter (or q to quit).
      - take failure (arm/start/stop/de-arm error, or no files on the TDA):
        the worker stops framing and de-arms the TDA (a failed start used to
        leave the chips streaming into the SSD), reports the failure, exits,
        and the front-end re-initializes the radar before the next prompt.
        A failed take is NOT retried automatically, since that would move
        the arm again. Type it again at the prompt.

Motion: when mmw_start_frame() returns, the worker runs that motion through
ur_ws/motion_traj.py inside distrobox (default: ros2):

    distrobox enter --name ros2 --no-tty -- python -u motion_traj.py <script> \\
        --output robot_trajectories/<capture_dir>_pos-timestamps.npy

so the arm starts moving as soon as the TDA framing window is live, and the
TCP trajectory ([t, x, y, z, rx, ry, rz] at ~100 Hz) is recorded for that
window and SCP'd into /mnt/ssd/<capture_dir>/ afterwards. The motion's own
output goes to logs/<capture_dir>_motion.log and is echoed as "[ARM]" lines.
Motion scripts live in --motion-dir (default: this repo's ur_ws/, else
~/Projects/ur_ws). Type a short name (circle), a stem (motion_circle), or a
filename (motion_circle.py). Any extra argv after the name is passed through.

Ctrl+C: at the prompt, exits. During a take, the first Ctrl+C only warns.
A second one stops the radar right away (stop + de-arm). The arm is never
stopped from here: the motion runs in its own session and finishes its
trajectory.

Same hardware caveats as mimo_interactive.py (wall-clock frame counts with
RF numFrames=0; SSD/network drift on long same-process sessions).

Examples:
    python3 mimo_robotic.py --frames 2000 --distrobox ros2
    python3 mimo_robotic.py --frames 2000 -c cascade_tx5_rx16_robotic -v
"""
import argparse
import copy
import glob
import json
import os
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime

# Scripts do not auto-import GNU readline; without it, input() treats arrow
# keys as escape sequences instead of moving the cursor / recalling history.
try:
    import readline  # noqa: F401
except ImportError:
    pass

from radar_config import RADAR_CONFIGS

DEFAULT_RADAR_CONFIG = "cascade_tx7_rx16_5rps_ns128"
DEFAULT_DISTROBOX = "ros2"
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_UR_WS = os.path.join(_HERE, "ur_ws")
DEFAULT_MOTION_DIR = (
    _REPO_UR_WS if os.path.isdir(_REPO_UR_WS)
    else os.path.expanduser("~/Projects/ur_ws")
)
TRAJ_DIR = "robot_trajectories"
EVENT_PREFIX = "@MRB "
EXIT_NEEDS_RESTART = 3
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")

# mmwavelink return codes (ti/mmwavelink/mmwavelink.h) worth explaining.
RL_ERRORS = {
    -1: "mmWaveLink protocol error",
    -2: "invalid input",
    -4: "radar HW/SW interface error",
    -6: "CRC mismatch",
    -7: "checksum mismatch",
    -8: "device did not answer in time (RESP_TIMEOUT)",
    -9: "fatal error inside the device",
    -16: "device sent NACK",
    -17: "host IRQ timeout",
    20: "frames already started",
    21: "frames already stopped",
}


# --------------------------------------------------------------------------
# Motion helpers (shared by front-end validation and the worker)
# --------------------------------------------------------------------------

def _list_motion_scripts(motion_dir):
    """Basenames of motion_*.py under motion_dir, excluding the recorder wrapper."""
    if not os.path.isdir(motion_dir):
        return []
    return sorted(
        os.path.basename(p)
        for p in glob.glob(os.path.join(motion_dir, "motion_*.py"))
        if os.path.basename(p) != "motion_traj.py"
    )


def _venv_python(motion_dir):
    """Prefer the ur_ws RTDE venv so rtde_control imports without bashrc."""
    candidate = os.path.join(motion_dir, ".ur_rtde_env", "bin", "python")
    if os.path.isfile(candidate):
        return candidate
    return "python"


# Prompt aliases: alias -> short name of the motion_*.py it runs. The alias is
# also the label in the TDA directory name (horz_<exp_name>_<timestamp>).
MOTION_ALIASES = {
    "horz": "line_horz",
    "vert": "line_vert",
}


def _motion_short_name(filename):
    """motion_circle.py -> circle; motion_line_horz.py -> horz (alias); other stems unchanged."""
    stem = os.path.basename(filename)
    stem = stem[:-3] if stem.endswith(".py") else stem
    if stem.startswith("motion_"):
        stem = stem[len("motion_"):]
    for alias, target in MOTION_ALIASES.items():
        if stem == target:
            return alias
    return stem


def motion_label(motion_line, motion_dir):
    """Short label of the motion a prompt names ('line_horz', 'horz', 'motion_line_horz.py' -> 'horz')."""
    return _motion_short_name(resolve_motion_argv(motion_line, motion_dir)[2])


def resolve_motion_script(name, motion_dir):
    """
    Resolve a prompt token to a motion_*.py path.

    Accepts: circle, motion_circle, motion_circle.py, or an absolute path.
    """
    raw = MOTION_ALIASES.get(name, name)
    candidates = []
    if os.path.isabs(raw):
        candidates.append(raw)
    else:
        base = os.path.basename(raw)
        base_py = base if base.endswith(".py") else base + ".py"
        candidates.append(os.path.join(motion_dir, base_py))
        if not base_py.startswith("motion_"):
            candidates.append(os.path.join(motion_dir, "motion_" + base_py))
        else:
            candidates.append(os.path.join(motion_dir, base_py[len("motion_"):]))

    allowed = {os.path.abspath(os.path.join(motion_dir, s))
               for s in _list_motion_scripts(motion_dir)}
    for path in candidates:
        resolved = os.path.abspath(path)
        if resolved in allowed and os.path.isfile(resolved):
            return resolved

    available = ", ".join(_motion_short_name(s)
                          for s in _list_motion_scripts(motion_dir))
    raise FileNotFoundError(f"Unknown motion '{name}'. Available: {available}")


def resolve_motion_argv(motion_line, motion_dir):
    """
    Turn a prompt motion string into argv to run *inside* distrobox.

    Accepts:
      circle
      motion_circle
      motion_circle.py
      python motion_circle.py
      python -u circle --radius 0.2
    """
    tokens = shlex.split(motion_line)
    if not tokens:
        raise ValueError("empty motion command")

    i = 0
    if os.path.basename(tokens[0]) in ("python", "python3"):
        i = 1
        if i < len(tokens) and tokens[i] == "-u":
            i += 1
    if i >= len(tokens):
        raise ValueError(f"motion command has no script: {motion_line!r}")

    script = resolve_motion_script(tokens[i], motion_dir)
    extra = tokens[i + 1:]
    return [_venv_python(motion_dir), "-u", script] + extra


def wrap_motion_traj(inner_argv, motion_dir, output_path):
    """Run the resolved motion through motion_traj.py so TCP pose is recorded."""
    traj = os.path.join(motion_dir, "motion_traj.py")
    if not os.path.isfile(traj):
        raise FileNotFoundError(f"motion_traj.py not found in {motion_dir}")
    python = inner_argv[0]
    tokens = inner_argv[1:]
    if tokens and tokens[0] == "-u":
        tokens = tokens[1:]
    script = tokens[0]
    extra = tokens[1:]
    if os.path.basename(script) == "motion_traj.py":
        return [python, "-u", traj, *extra, "--output", output_path]
    return [python, "-u", traj, script, "--output", output_path, *extra]


def distrobox_cmd(distrobox_name, inner_argv):
    return ["distrobox", "enter", "--name", distrobox_name, "--no-tty",
            "--"] + inner_argv


def warmup_distrobox(distrobox_name, ui):
    """Start the container now so the later enter is not paid at frame start."""
    cmd = distrobox_cmd(distrobox_name, ["true"])
    ui.info(f"warming distrobox '{distrobox_name}' ...")
    try:
        result = subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, text=True)
    except FileNotFoundError:
        ui.fail("distrobox not found on PATH. Install distrobox, or this host "
                "is not the machine that owns the UR workspace.")
        sys.exit(1)
    if result.returncode != 0:
        ui.fail(f"distrobox enter '{distrobox_name}' failed (exit {result.returncode}). "
                "Check `distrobox list`.")
        if result.stderr.strip():
            ui.dim(result.stderr.strip())
        sys.exit(1)
    ui.ok(f"distrobox '{distrobox_name}' ready")


class MotionJob:
    """Worker side: spawn motion_traj.py at frame start; wait(); upload the .npy.

    The motion's stdout/stderr go to a file (not a pipe into this worker), so
    the arm script can never die of a broken pipe if this worker exits
    mid-move. A tail thread echoes that file as "[ARM] ..." lines.
    """

    def __init__(self, distrobox_name, inner_argv, motion_dir, tda_ip):
        self.distrobox_name = distrobox_name
        self.inner_argv = inner_argv
        self.motion_dir = motion_dir
        self.tda_ip = tda_ip
        self.proc = None
        self.rc = None
        self.traj_path = None
        self.log_path = None
        self._tail = None

    def start(self, capture_dir=None):
        name = capture_dir or datetime.now().strftime("traj_%y%m%d_%H%M%S")
        os.makedirs(TRAJ_DIR, exist_ok=True)
        os.makedirs("logs", exist_ok=True)
        self.traj_path = os.path.abspath(
            os.path.join(TRAJ_DIR, f"{name}_pos-timestamps.npy"))
        self.log_path = os.path.abspath(os.path.join("logs", f"{name}_motion.log"))
        try:
            wrapped = wrap_motion_traj(self.inner_argv, self.motion_dir, self.traj_path)
        except FileNotFoundError as e:
            print(f"[ROBOT] WARNING: {e}; launching motion without trajectory recording.")
            wrapped = self.inner_argv
            self.traj_path = None
        cmd = distrobox_cmd(self.distrobox_name, wrapped)
        print(f"[ROBOT] frames started - launching: {shlex.join(cmd)}")
        try:
            with open(self.log_path, "wb") as log:
                # Own session: signals aimed at the radar must not reach the arm.
                self.proc = subprocess.Popen(
                    cmd, cwd=self.motion_dir, start_new_session=True,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        except OSError as e:
            print(f"[ROBOT] ERROR: failed to launch motion ({e}); capture continues.")
            self.proc = None
            return
        self._tail = threading.Thread(target=self._tail_log, daemon=True)
        self._tail.start()

    def _tail_log(self):
        with open(self.log_path, "r", errors="replace") as f:
            pending = ""
            while True:
                chunk = f.readline()
                if chunk:
                    pending += chunk
                    if pending.endswith("\n"):
                        print(f"[ARM] {pending.rstrip()}", flush=True)
                        pending = ""
                    continue
                if self.proc.poll() is not None:
                    if pending:
                        print(f"[ARM] {pending.rstrip()}", flush=True)
                    return
                time.sleep(0.1)

    def wait_until_done(self, fallback_s):
        """wait_fn for run_one_capture(): block until the motion exits.

        fallback_s is used only if the motion process never started (then we
        wait the --frames window so the capture is not zero-length).
        """
        if self.proc is None:
            print("[ROBOT] WARNING: motion did not start; waiting the --frames window.")
            time.sleep(fallback_s)
            return
        print("[ROBOT] capturing until motion ends...")
        self.rc = self.proc.wait()
        print(f"[ROBOT] motion {'finished' if self.rc == 0 else f'exited {self.rc}'} - stopping radar.")

    def wait(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                print("[ROBOT] waiting for motion to finish...")
            self.rc = self.proc.wait()
        if self._tail is not None:
            self._tail.join(timeout=2.0)

    def upload(self, capture_dir):
        """Upload trajectory + motion log. Returns True/False for the trajectory, None if none."""
        if not capture_dir:
            return None
        files = [p for p in (self.log_path,) if p and os.path.isfile(p)]
        traj_ok = None
        if self.traj_path:
            if os.path.isfile(self.traj_path):
                files.insert(0, self.traj_path)
                traj_ok = True
            else:
                print(f"[ROBOT] WARNING: trajectory .npy not written ({self.traj_path}).")
                traj_ok = False
        if files:
            import mimo  # its upload_files_to_tda, so worker_take() records the result
            if not mimo.upload_files_to_tda(files, capture_dir, self.tda_ip) and traj_ok:
                print("[ROBOT] WARNING: trajectory upload to the TDA failed (kept locally).")
                traj_ok = False
        return traj_ok


# --------------------------------------------------------------------------
# Worker process (--worker): owns the radar
# --------------------------------------------------------------------------

class Hangup(Exception):
    """Raised from the SIGTERM/SIGHUP handler to unwind a take and clean up."""


class Events:
    """Writes @MRB events to a duplicate of the stdout fd this process started with
    (taken before any TeeLogger, so events never land in the per-take log)."""

    def __init__(self):
        sys.stdout.flush()
        self._fd = os.dup(1)

    def send(self, event, **fields):
        line = "\n" + EVENT_PREFIX + json.dumps({"event": event, **fields},
                                                separators=(",", ":")) + "\n"
        try:
            os.write(self._fd, line.encode())
        except OSError:
            pass


def worker_take(exp_name, motion_line, args, events):
    """Record one take. Returns True when the worker must exit (radar needs a fresh init)."""
    import mimo
    import mmwcas
    from utility import TeeLogger
    from mimo_zed_slave import RadarCalls, _frames_from_files, _frame_check, _stop_radar

    motion = None
    if motion_line:
        try:
            motion = MotionJob(args.distrobox, resolve_motion_argv(motion_line, args.motion_dir),
                               args.motion_dir, args.tda_ip)
        except (ValueError, FileNotFoundError) as e:
            events.send("done", ok=False, stage="motion", capture_dir="", message=str(e),
                        needs_restart=False)
            return False

    radar = RadarCalls(mmwcas)
    checked = {"files": None}
    uploads = {}
    original_check = mimo.check_captured_files
    original_upload = mimo.upload_files_to_tda

    def upload_files_to_tda(paths, *rest, **kwargs):
        result = original_upload(paths, *rest, **kwargs)
        for path in paths:
            if path:
                uploads[os.path.basename(path)] = bool(result)
        return result

    def check_captured_files(capture_dir, tda_ip, *rest, **kwargs):
        result = original_check(capture_dir, tda_ip, *rest, **kwargs)
        checked["files"] = result
        return result

    def on_frame_start(capture_dir):
        events.send("capturing", capture_dir=capture_dir, motion=motion_line or None,
                    wait_s=None if motion else args.wait_s)
        if motion is not None:
            motion.start(capture_dir)

    tee = None
    if not args.no_log:
        provisional = os.path.join(
            "logs", f"mimo_robotic_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
        try:
            tee = TeeLogger(provisional).start()
        except (OSError, ImportError) as exc:
            print(f"[LOG] WARNING: terminal logging unavailable ({exc}); continuing without it.")

    status, capture_dir, message, hangup = None, None, "", False
    mimo.mmwcas = radar
    mimo.check_captured_files = check_captured_files
    mimo.upload_files_to_tda = upload_files_to_tda
    try:
        status, capture_dir = mimo.run_one_capture(
            f"{motion_label(motion_line, args.motion_dir)}_{exp_name}" if motion else exp_name,
            args.frames, args.tda_ip, on_frame_start=on_frame_start,
            wait_fn=(lambda: motion.wait_until_done(args.wait_s)) if motion else None)
    except Hangup:
        hangup, message = True, "stopped by user"
    except Exception as exc:
        message = f"run_one_capture raised {exc!r}"
    finally:
        mimo.mmwcas = mmwcas
        mimo.check_captured_files = original_check
    hangup = hangup or args.hung_up

    files = (checked["files"] or (False, 0, []))[2]
    if message:
        stage = "stopped" if hangup else "exception"
    elif radar.status_of("mmw_arming_tda") not in (None, 0):
        stage = "arm"
    elif radar.status_of("mmw_start_frame") not in (None, 0):
        stage = "start"
    elif radar.status_of("mmw_stop_frame") not in (None, 0):
        stage = "stop"
    elif radar.status_of("mmw_dearming_tda") not in (None, 0):
        stage = "dearm"
    elif checked["files"] is None or not checked["files"][0]:
        stage = "verify"
        message = "no capture files appeared on the TDA"
    else:
        stage = "ok"
    ok = stage == "ok" and status == 0
    if not ok and not message:
        code = f"status {status}" + (f" - {RL_ERRORS[status]}" if status in RL_ERRORS else "")
        fn = {"arm": "mmw_arming_tda", "start": "mmw_start_frame", "stop": "mmw_stop_frame",
              "dearm": "mmw_dearming_tda"}.get(stage)
        message = f"{fn} failed ({code})" if fn else code

    if not ok:
        print(f"[ROBOT] ERROR: take failed at stage '{stage}': {message}. "
              "Stopping the radar and de-arming the TDA.")
        _stop_radar()

    traj_ok = None
    if motion is not None:
        motion.wait()
        traj_ok = motion.upload(capture_dir) if (capture_dir and stage not in ("arm", "start")) else None

    capture_dir = capture_dir or ""
    log_path = None
    try:
        mimo._finish_log(tee, capture_dir or None, ok, args.tda_ip)
        if tee is not None and capture_dir:
            log_path = os.path.join("logs", f"{capture_dir}.log")
    except Exception as exc:
        print(f"[LOG] WARNING: finishing the log failed ({exc!r})")

    mimo.upload_files_to_tda = original_upload
    events.send(
        "done", ok=ok, status=status, stage=stage, capture_dir=capture_dir, message=message,
        frames_idx=_frames_from_files(files),
        frame_check=_frame_check(capture_dir, args.period_ms, args.frames) if ok else None,
        files=len(files), log=log_path,
        report=_file_report(files, mimo.config_dict), uploads=dict(uploads),
        motion=motion_line or None, motion_rc=motion.rc if motion else None, traj_ok=traj_ok,
        needs_restart=not ok,
    )
    return hangup or not ok


def _file_report(files, cfg):
    """Per-device rows for the front-end's file check: frames the TDA indexed and payload size.

    A device's data can span several <dev>_<nnnn>_data.bin files (pre-allocated 2 GB
    each), so rows are grouped by device; payload = indexed frames x bytes/frame.
    """
    import mimo
    rx = mimo._rx_popcount_per_device(cfg)
    devs = {}
    for name, size in files:
        if not str(size).isdigit():
            continue
        dev = mimo._dev_id_from_data_bin(name)
        if dev is None:
            continue
        row = devs.setdefault(dev, {"dev": dev, "rx": rx[dev] if dev < len(rx) else None,
                                    "frames": None, "bytes": 0, "nfiles": 0})
        if name.endswith("_idx.bin"):
            row["frames"] = mimo._frames_from_idx_size(int(size))
        elif name.endswith("_data.bin"):
            row["bytes"] += int(size)
            row["nfiles"] += 1
    for row in devs.values():
        bpf = mimo._expected_bytes_per_frame_for_device(cfg, row["dev"])
        if bpf and row["frames"] is not None:
            row["bytes"] = row["frames"] * bpf
    rows = [devs[d] for d in sorted(devs) if devs[d]["nfiles"]]
    other = sorted(n for n, _ in files if not n.endswith(("_data.bin", "_idx.bin")))
    return {"rows": rows, "other": other}


def worker_main(args):
    import mimo
    import mmwcas
    from radar_config import get_radar_config

    events = Events()
    args.hung_up = False

    def on_signal(signum, _frame):
        if not args.hung_up:  # once only, so a second signal cannot abort the stop/de-arm cleanup
            args.hung_up = True
            raise Hangup(signal.Signals(signum).name)

    signal.signal(signal.SIGHUP, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    mimo.config_dict = copy.deepcopy(get_radar_config(args.radar_config))
    # RF chips stay at numFrames=0 (infinite): each take's length is set by the
    # host-side wait before mmw_stop_frame(), see mimo_interactive.py.
    mimo.config_dict["mimo"]["frame"]["numFrames"] = 0
    args.period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    args.wait_s = mimo._wait_s_for_frames(args.frames, mimo._frame_period_s(mimo.config_dict))
    events.send("hello", pid=os.getpid())

    code = 1
    try:
        status = mmwcas.mmw_set_config(mimo.config_dict)
        if status != 0:
            events.send("fatal", retry=False, message=f"mmw_set_config failed (status {status})")
            return 2
        status = mmwcas.mmw_init(args.tda_ip)
        if status != 0:
            events.send("fatal", retry=True, message=f"mmw_init failed (status {status})")
            return 2
        time.sleep(2)
        os.makedirs("mmwave_json_files", exist_ok=True)
        if args.no_ir:
            print("[IR] IR sensor timestamp logging disabled (--no-ir).")
        else:
            mimo.setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
        events.send("ready", period_ms=args.period_ms, wait_s=args.wait_s)

        while True:
            line = sys.stdin.readline()
            if not line:
                return 0
            try:
                cmd = json.loads(line)
            except ValueError:
                continue
            if cmd.get("cmd") == "exit":
                return 0
            if cmd.get("cmd") == "capture":
                if worker_take(cmd["exp"], cmd.get("motion") or "", args, events):
                    return EXIT_NEEDS_RESTART
                events.send("ready", period_ms=args.period_ms, wait_s=args.wait_s)
    except Hangup:
        _stop_radar_quiet()
        code = 0
    except Exception as exc:
        print(f"[ROBOT] ERROR: worker fatal: {exc!r}")
        events.send("fatal", retry=True, message=repr(exc))
    finally:
        mimo.teardown_ir_sensor()
    return code


def _stop_radar_quiet():
    from mimo_zed_slave import _stop_radar
    _stop_radar()


# --------------------------------------------------------------------------
# Front-end: compact coloured UI + worker supervision
# --------------------------------------------------------------------------

_STATUS_RE = re.compile(r"^STATUS\s+(-?\d+)\s*\|\s*DEV MAP:\s*(\d+)\s*\|\s*(.*)$")
# Error lines that only restate what the final result line says.
_SUMMARIZED_RE = re.compile(r"^mmw_\w+ failed \(status|take failed at stage|^STATUS\s+-?\d+"
                            r"|frame\(s\) short of|No files found in capture|data capture failed")
_PROBLEM_RE = re.compile(r"error|fail|warning|traceback|exception|dropped|timed? ?out|not found",
                         re.IGNORECASE)


class UI:
    def __init__(self, color):
        self.color = color
        self._progress = False
        self.lock = threading.Lock()

    def _c(self, code, text):
        return f"\033[{code}m{text}\033[0m" if self.color else text

    def _out(self, text):
        with self.lock:
            if self._progress:
                sys.stdout.write("\r\033[K" if self.color else "\n")
                self._progress = False
            sys.stdout.write(text + "\n")
            sys.stdout.flush()

    def progress(self, text):
        if not self.color:
            return  # no in-place redraw without a terminal; keep output short instead
        width = max(20, (os.get_terminal_size().columns if sys.stdout.isatty() else 100) - 1)
        with self.lock:
            sys.stdout.write("\r\033[K" + self._c("2", text[:width]))
            sys.stdout.flush()
            self._progress = True

    def ok(self, text):    self._out(self._c("32", "✓ ") + text)
    def fail(self, text):  self._out(self._c("1;31", "✗ " + text))
    def warn(self, text):  self._out(self._c("33", "! " + text))
    def info(self, text):  self._out(self._c("36", "• ") + text)
    def dim(self, text):   self._out(self._c("2", text))
    def raw(self, text):   self._out(text)
    def bold(self, text):  return self._c("1", text)

    def prompt(self, text):
        """Bold prompt for input(). The colour codes are wrapped in \\001/\\002 so
        readline knows they take no screen width - without that, line editing
        (backspace, arrows, Ctrl+U) lands on the wrong column."""
        return f"\001\033[1m\002{text}\001\033[0m\002" if self.color else text


def _explain_status(line):
    m = _STATUS_RE.match(line)
    if not m:
        return line
    status, dev, msg = int(m.group(1)), int(m.group(2)), m.group(3)
    why = f" - {RL_ERRORS[status]}" if status in RL_ERRORS else ""
    return f"{msg} (status {status}{why}, dev map {dev})"


class Worker:
    """One radar worker process on a pty, so C printf stays line-buffered and live."""

    def __init__(self, args, ui, session_log):
        import pty
        import termios
        self.ui, self.args, self.session_log = ui, args, session_log
        self.events = queue.Queue()
        self.tail = deque(maxlen=40)
        self.last_error = None
        self.phase = "init"
        self.init_steps = 0

        master, slave = pty.openpty()
        try:
            attrs = termios.tcgetattr(slave)
            attrs[1] &= ~termios.ONLCR
            attrs[3] &= ~termios.ECHO
            termios.tcsetattr(slave, termios.TCSANOW, attrs)
        except termios.error:
            pass
        cmd = [sys.executable, "-u", os.path.abspath(__file__), "--worker",
               "-f", str(args.frames), "-c", args.radar_config, "--tda-ip", args.tda_ip,
               "--ir-pin", str(args.ir_pin), "--ir-bounce-ms", str(args.ir_bounce_ms),
               "--distrobox", args.distrobox, "--motion-dir", args.motion_dir]
        cmd += ["--no-ir"] if args.no_ir else []
        cmd += ["--no-log"] if args.no_log else []
        # Own session: Ctrl+C in the terminal reaches only the front-end.
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=slave, stderr=slave,
                                     start_new_session=True, text=True, bufsize=1)
        os.close(slave)
        self._master = master
        threading.Thread(target=self._reader, daemon=True).start()

    # ---- output handling -------------------------------------------------
    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = os.read(self._master, 4096)
            except OSError:
                chunk = b""
            if not chunk:
                break
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                self._handle(raw.decode(errors="replace").rstrip("\r"))
        if buf:
            self._handle(buf.decode(errors="replace").rstrip("\r"))
        os.close(self._master)
        self.events.put(("eof", None))

    def _handle(self, line):
        if EVENT_PREFIX in line:
            before, _, payload = line.partition(EVENT_PREFIX)
            try:
                self.events.put(("event", json.loads(payload)))
            except ValueError:
                pass
            if not before.strip():
                return
            line = before
        self.session_log.write(line + "\n")
        self.session_log.flush()
        if not line.strip():
            return
        self.tail.append(line)

        status = _STATUS_RE.match(line)
        problem = bool(status and int(status.group(1)) != 0) or (
            not status and _PROBLEM_RE.search(line))
        if problem:
            self.last_error = line
        if self.args.verbose:
            self.ui.raw(line)
            return
        if self.phase == "init":
            # Radar bring-up is shown in full, one tidy line per step.
            if status and not problem:
                self.init_steps += 1
                self.ui.dim(f"  ✓ {status.group(3)}  (dev map {status.group(2)})")
            elif problem:
                self.ui.fail(_explain_status(line) if status else line.strip())
            else:
                self.ui.dim("    " + line.strip())
            return
        if status and not problem:
            return
        if line.startswith("[ARM]"):
            self.ui.dim("  " + line)
        elif problem and _SUMMARIZED_RE.search(line):
            return  # the one-line take result already reports this
        elif problem:
            text = _explain_status(line) if status else line.strip()
            is_error = status or (not re.search(r"warning", line, re.I)
                                  and re.search(r"error|fail|traceback", line, re.I))
            (self.ui.fail if is_error else self.ui.warn)(text)

    # ---- control ----------------------------------------------------------
    def send(self, obj):
        try:
            self.proc.stdin.write(json.dumps(obj) + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass

    def next_event(self, on_interrupt=None):
        """Block for the next worker event; ("eof", None) when the worker is gone."""
        while True:
            try:
                return self.events.get(timeout=0.25)
            except queue.Empty:
                continue
            except KeyboardInterrupt:
                if on_interrupt is None:
                    raise
                on_interrupt()

    def alive(self):
        return self.proc.poll() is None

    def stop_radar_now(self):
        if self.alive():
            self.proc.send_signal(signal.SIGTERM)

    def close(self, timeout=15.0):
        if self.alive():
            self.send({"cmd": "exit"})
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
        try:
            self.proc.stdin.close()
        except OSError:
            pass


def start_radar(args, ui, session_log):
    """Start a worker and wait until the radar is configured. Re-flashes on failure.

    Returns a ready Worker, or None if the user chose to quit.
    """
    attempt = 0
    while True:
        attempt += 1
        label = "initializing radar" if attempt == 1 else f"re-flashing radar (attempt {attempt})"
        ui.info(f"{label} - power up, firmware download, RF config ...")
        t0 = time.time()
        worker = Worker(args, ui, session_log)
        fatal = None
        try:
            while True:
                kind, ev = worker.next_event()
                if kind == "eof":
                    break
                if ev["event"] == "ready":
                    worker.phase = "idle"
                    ui.ok(f"radar ready  ({worker.init_steps} steps ok, {time.time() - t0:.0f}s)")
                    return worker
                if ev["event"] == "fatal":
                    fatal = ev
        except KeyboardInterrupt:
            worker.stop_radar_now()
            worker.close(timeout=5.0)
            ui.warn("interrupted during radar init")
            return None
        worker.proc.wait()
        reason = fatal["message"] if fatal else (
            _explain_status(worker.last_error) if worker.last_error
            else f"worker exited with code {worker.proc.returncode}")
        ui.fail(f"radar init failed: {reason}")
        if fatal and not fatal.get("retry", True):
            ui.dim("  this is a configuration error - fix the preset, re-flashing will not help.")
            return None
        if attempt <= args.init_retries:
            ui.warn(f"re-flashing in 3 s ... ({attempt}/{args.init_retries})")
            try:
                time.sleep(3)
            except KeyboardInterrupt:
                return None
            continue
        ui.warn("still failing after re-flash. Power-cycle the cascade board "
                "(12 V off ~10 s), wait for the TDA to boot, then press Enter.")
        try:
            answer = input(ui.prompt("Enter = retry, q = quit > ")).strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if answer in ("q", "quit", "exit"):
            return None
        attempt = 0


def run_take(worker, exp_name, motion_line, ui):
    """Send one take to a ready worker and print the compact result. Returns the done event or None."""
    worker.last_error = None
    worker.send({"cmd": "capture", "exp": exp_name, "motion": motion_line})
    worker.phase = "take"
    presses = []
    t_start = [None]

    def on_interrupt():
        presses.append(time.time())
        if len(presses) == 1:
            ui.warn("take in progress - Ctrl+C again to STOP THE RADAR NOW "
                    "(the arm is not stopped and finishes its motion)")
        elif len(presses) == 2:
            ui.warn("stopping radar ...")
            worker.stop_radar_now()

    done = None
    while True:
        kind, ev = worker.next_event(on_interrupt)
        if kind == "eof":
            break
        if ev["event"] == "capturing":
            t_start[0] = time.time()
            what = (f"motion '{ev['motion']}' (radar stops when it ends)" if ev.get("motion")
                    else f"radar only, ~{ev['wait_s']:.0f}s")
            ui.info(f"recording {ui.bold(ev['capture_dir'])}  - {what}")
        elif ev["event"] == "done":
            done = ev
        elif ev["event"] == "ready":
            break
    worker.phase = "idle"

    if done is None:
        worker.proc.wait()
        if len(presses) >= 2:
            ui.warn("radar stopped by user (stop + de-arm sent); this take is incomplete")
            return None
        reason = _explain_status(worker.last_error) if worker.last_error else \
            f"worker exited with code {worker.proc.returncode}"
        ui.fail(f"take aborted - radar process died: {reason}")
        for line in list(worker.tail)[-6:]:
            ui.dim("    " + line)
        return None

    dur = f"{time.time() - t_start[0]:.1f}s" if t_start[0] else "-"
    name = done.get("capture_dir") or exp_name
    if done["ok"]:
        fc = done.get("frame_check") or {}
        frames = fc.get("frames") or max((done.get("frames_idx") or {}).values(), default=None)
        parts = [f"{frames} frames" if frames is not None else "frames ?", dur]
        gaps = fc.get("gaps")
        if gaps:
            parts.append(f"{gaps} gap(s), {fc['skipped']} skipped, "
                         f"largest {fc['largest_gap_ms']} ms @ frame {fc['first_gap_frame']}")
        if done.get("motion"):
            rc = done.get("motion_rc")
            parts.append("motion ok" if rc == 0 else f"motion exit {rc}")
            parts.append({True: "traj ✓", False: "traj ✗", None: "no traj"}[done.get("traj_ok")])
        line = f"{ui.bold(name)}  " + "  ".join(parts)
        if gaps or (done.get("motion") and (done.get("motion_rc") != 0 or done.get("traj_ok") is False)):
            ui.warn(line)
        else:
            ui.ok(line)
    else:
        detail = _explain_status(worker.last_error) if (
            worker.last_error and _STATUS_RE.match(worker.last_error)) else None
        ui.fail(f"{name}  FAILED at '{done['stage']}': {done['message']}")
        if detail and done.get("stage") not in ("motion", "stopped"):
            ui.dim(f"    radar said: {detail}")
    _print_file_report(done, ui)
    if done.get("log"):
        ui.dim(f"  log: {done['log']}")
    return done


def _human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024.0


def _print_file_report(done, ui):
    """Clean summary of what landed on the TDA SSD for this take."""
    report = done.get("report") or {}
    rows = report.get("rows") or []
    if not done.get("capture_dir") or (not rows and done.get("stage") in ("arm", "start", "motion")):
        return
    names = {0: "master", 1: "slave1", 2: "slave2", 3: "slave3"}
    ui.dim(f"  TDA /mnt/ssd/{done['capture_dir']}/  ({done.get('files', 0)} files)")
    if not rows:
        ui.fail("    no *_data.bin on the SSD")
        return
    counts = [r["frames"] for r in rows if r["frames"] is not None and r["rx"]]
    ref = max(counts) if counts else None
    requested = None if done.get("motion") else (done.get("frame_check") or {}).get("requested")
    for r in rows:
        label = names.get(r["dev"], f"dev{r['dev']}")
        frames = "-" if r["frames"] is None else str(r["frames"])
        text = f"    {label:<7} {str(r['rx'] or 0) + ' RX':<5} {frames:>6} frames  {_human(r['bytes']):>9}"
        if not r["rx"]:
            ui.dim(text + "  (RX off)")
        elif r["frames"] is None or (ref is not None and r["frames"] != ref) or \
                (requested and r["frames"] < requested):
            short = f"  <- {requested - r['frames']} short of {requested}" if (
                requested and r["frames"] is not None and r["frames"] < requested) else "  <- mismatch"
            ui.warn(text[2:] + short)
        else:
            ui.raw(text)
    total = sum(r["bytes"] for r in rows)
    fc = done.get("frame_check") or {}
    line = f"    total {_human(total)}"
    if fc:
        line += (f"  |  frame timing: {fc['gaps']} gap(s), {fc['skipped']} skipped"
                 if fc.get("gaps") else "  |  frame timing: uniform, no gaps")
    ui.raw(line)
    uploads = done.get("uploads") or {}
    if uploads:
        def short(name):
            for key, lab in (("_rf_frame_timestamps", "rf_ts"), ("_rpi_frame_timestamps", "rpi_ts"),
                             ("_ir_timestamps", "ir_ts"), (".mmwave.json", "json"),
                             ("_pos-timestamps", "traj"), ("_motion.log", "motion_log"), (".log", "log")):
                if key in name:
                    return lab
            return name
        marks = "  ".join(f"{short(n)} {'✓' if ok else '✗'}" for n, ok in uploads.items())
        (ui.raw if all(uploads.values()) else ui.warn)(f"    sidecars: {marks}")


def config_summary(cfg):
    """A few lines describing the preset: chirp, TX/RX geometry, frame, derived numbers."""
    prof, frame, chan, chirp = (cfg["mimo"][k] for k in ("profile", "frame", "channel", "chirp"))
    n_adc, fs_ksps = int(prof["numAdcSamples"]), float(prof["adcSamplingFrequency"])
    slope, ramp = float(prof["frequencySlope"]), float(prof["rampEndTime"])
    idle = prof.get("idleTimes", [prof.get("idleTime", 0)])
    n_chirps = int(frame["chirpEndIdx"]) - int(frame["chirpStartIdx"]) + 1
    loops, period_ms = int(frame["numLoops"]), float(frame["framePeriodicity"])
    rx_en = chan["rxChannelEn"]
    rx_en = rx_en if isinstance(rx_en, list) else [rx_en] * 4
    rx = [bin(int(m)).count("1") for m in rx_en]
    table = chirp.get("txAntennaTable") or []
    tx_order = []
    for c in range(n_chirps):
        fired = [f"TX{dev * 3 + row[c] + 1}" for dev, row in enumerate(table)
                 if c < len(row) and row[c] >= 0]
        tx_order.append("+".join(fired) or "-")
    pids = chirp.get("profileIdPerChirp") or [0] * n_chirps
    chirp_us = [float(idle[min(int(p), len(idle) - 1)]) + ramp for p in pids[:n_chirps]]
    loop_us = sum(chirp_us)
    active_ms = loop_us * loops / 1000.0
    bw_hz = slope * 1e12 * (n_adc / (fs_ksps * 1e3))
    c0 = 299792458.0
    rate = n_adc * n_chirps * loops * sum(rx) * 4 / (period_ms / 1000.0)
    return [
        f"chirp   {prof['startFrequency']} GHz, {slope:g} MHz/us, {n_adc} samples @ {fs_ksps:g} ksps, "
        f"ramp {ramp:g} us, idle {'/'.join(f'{float(i):g}' for i in idle)} us, rx gain {prof['rxGain']} dB",
        f"tx      {n_chirps} chirps/loop: {' '.join(tx_order)}",
        f"rx      {sum(rx)} RX  (per device {'/'.join(map(str, rx))})  ->  {n_chirps * sum(rx)} virtual",
        f"frame   {loops} loops x {loop_us:.1f} us = {active_ms:.2f} ms active / {period_ms:g} ms period "
        f"({100 * active_ms / period_ms:.0f}% duty, {1000 / period_ms:.1f} fps)",
        f"derived BW {bw_hz / 1e9:.2f} GHz, range res {100 * c0 / (2 * bw_hz):.1f} cm, "
        f"max range {fs_ksps * 1e3 * c0 / (2 * slope * 1e12):.2f} m, ~{rate / 1e6:.0f} MB/s to the SSD",
    ]


def frontend_main(args):
    color = (not args.no_color and sys.stdout.isatty() and "NO_COLOR" not in os.environ)
    ui = UI(color)
    os.makedirs("logs", exist_ok=True)
    session_path = os.path.join("logs", f"mimo_robotic_session_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
    session_log = open(session_path, "a")

    # Frame period for the header only; the worker loads the preset itself.
    from radar_config import get_radar_config
    cfg = get_radar_config(args.radar_config)
    period_ms = float(cfg["mimo"]["frame"]["framePeriodicity"])
    ui.raw(ui.bold("=== mimo_robotic ===") +
           f"  config {args.radar_config}  |  {period_ms:.0f} ms/frame ({1000 / period_ms:.1f} fps)"
           f"  |  radar-only takes: {args.frames} frames (~{args.frames * period_ms / 1000:.1f}s)")
    try:
        for line in config_summary(cfg):
            ui.raw("  " + line)
    except (KeyError, TypeError, ValueError, IndexError, ZeroDivisionError) as e:
        ui.warn(f"could not summarize the preset ({e!r}) - see radar_configs/{args.radar_config}.toml")
    ui.dim(f"  distrobox {args.distrobox}  |  motions in {args.motion_dir}  |  session log {session_path}")

    warmup_distrobox(args.distrobox, ui)

    scripts = [_motion_short_name(s) for s in _list_motion_scripts(args.motion_dir)]
    worker = None
    shown_help = False
    try:
        while True:
            if worker is None or not worker.alive():
                if worker is not None:
                    worker.close()
                worker = start_radar(args, ui, session_log)
                if worker is None:
                    break
            if not shown_help:
                ui.dim("  type: <exp_name> [motion [args]]   e.g. 'test horz' -> horz_test_<stamp>, 'test' = radar only,"
                       " blank/quit = exit")
                if scripts:
                    ui.dim("  motions: " + ", ".join(scripts))
                shown_help = True
            try:
                line = input(ui.prompt("exp_name> ")).strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line or line in ("quit", "exit", "q"):
                break
            parts = line.split(maxsplit=1)
            exp_name, motion_line = parts[0], (parts[1] if len(parts) > 1 else "")
            if not _NAME_RE.match(exp_name):
                ui.fail(f"bad exp_name {exp_name!r} (letters, digits, . _ - only)")
                continue
            if motion_line:
                try:
                    resolve_motion_argv(motion_line, args.motion_dir)
                except (ValueError, FileNotFoundError) as e:
                    ui.fail(str(e))
                    continue
            done = run_take(worker, exp_name, motion_line, ui)
            if done is None or done.get("needs_restart") or not worker.alive():
                worker.close()
                worker = None
                if done is None or done.get("stage") != "stopped":
                    ui.warn("re-initializing the radar before the next take (re-flash). "
                            "The failed take is not repeated - type it again.")
    finally:
        if worker is not None:
            ui.dim("  shutting down radar ...")
            worker.close()
        session_log.close()
    ui.dim("Exiting robotic mode.")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-f', '--frames', type=int, default=100,
                        help='Frame count for radar-only takes (no motion on the prompt). '
                             'With a motion, the radar runs until the arm finishes. Default: 100.')
    parser.add_argument('-c', '--radar-config', type=str, default=DEFAULT_RADAR_CONFIG,
                        choices=sorted(RADAR_CONFIGS),
                        help=f"Preset from radar_configs/*.toml (default: '{DEFAULT_RADAR_CONFIG}').")
    parser.add_argument('--tda-ip', type=str, default='192.168.33.180',
                        help='TDA board IP address (default: 192.168.33.180)')
    parser.add_argument('--no-ir', action='store_true', help='Disable IR sensor timestamp logging.')
    parser.add_argument('--ir-pin', type=int, default=4,
                        help='BCM GPIO pin of the IR sensor (default: 4, matches receiver_ir.py)')
    parser.add_argument('--ir-bounce-ms', type=int, default=200,
                        help='IR sensor software debounce window in ms (default: 200)')
    parser.add_argument('--no-log', action='store_true',
                        help='Do not write/upload logs/<capture_dir>.log per take '
                             '(the front-end session log is always written).')
    parser.add_argument('--distrobox', type=str, default=DEFAULT_DISTROBOX,
                        help=f"Distrobox container that owns the UR RTDE env (default: '{DEFAULT_DISTROBOX}').")
    parser.add_argument('--motion-dir', type=str, default=DEFAULT_MOTION_DIR,
                        help=f"Directory of motion_*.py scripts (default: '{DEFAULT_MOTION_DIR}').")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Show the full mimo.py/mmwcas output instead of the compact summary.')
    parser.add_argument('--no-color', action='store_true', help='Plain text output.')
    parser.add_argument('--init-retries', type=int, default=3,
                        help='Automatic re-flash attempts when radar init fails, before asking '
                             'for a power-cycle (default: 3).')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)

    args = parser.parse_args()
    args.motion_dir = os.path.abspath(os.path.expanduser(args.motion_dir))
    if args.frames < 1:
        parser.error("--frames must be >= 1")
    if args.init_retries < 0:
        parser.error("--init-retries must be >= 0")

    if args.worker:
        return worker_main(args)
    return frontend_main(args)


if __name__ == "__main__":
    sys.exit(main())
