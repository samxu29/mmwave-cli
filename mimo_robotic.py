"""
mimo_robotic.py - interactive REPL for mimo.py's radar capture, with a
UR arm motion launched inside distrobox at the moment frames start
(TIDEP-01012 MIMO Cascade Radar, radar mounted on a UR3e TCP).

Same capture skeleton as mimo_interactive.py: configure the RF chips once,
then repeat named captures without re-launching the process. Frame count is
fixed for the whole session via --frames. Each prompt takes an exp_name
(same role as mimo.py --exp-name) and an optional motion command:

    exp_name> test  motion_line_horz
    exp_name> test  circle

A motion name launches the arm at mmw_start_frame() and STOPS the radar
when that motion process exits. --frames is the length of radar-only
captures (exp_name with no motion), not a motion timeout.

When mmw_start_frame() returns, this script runs that motion through
ur_ws/motion_traj.py inside distrobox (default: ros2):

    distrobox enter --name ros2 --no-tty -- python -u motion_traj.py <script> \\
        --output robot_trajectories/<capture_dir>_pos-timestamps.npy

so the arm starts moving as soon as the TDA framing window is live, not
during configure / clock-sync / TDA arm, and the TCP trajectory
([t, x, y, z, rx, ry, rz] at ~100 Hz) is recorded for the same window.
After the motion process exits, that .npy is SCP'd into
/mnt/ssd/<capture_dir>/ on the TDA alongside the other capture sidecars
(.mmwave.json, frame timestamps, terminal log).

Motion scripts live in --motion-dir (default: this repo's ur_ws/, else
~/Projects/ur_ws). Type a short name (circle), a stem (motion_circle), or
a filename (motion_circle.py); extra argv after the name is passed through.
The motion is chosen per prompt, never as a CLI flag.

The motion subprocess is in its own session so Ctrl+C on this REPL does
not SIGINT the arm mid-move. After the capture returns we wait for the
motion to finish before the next prompt.

Same hardware caveats as mimo_interactive.py (wall-clock frame counts with
RF numFrames=0; SSD/network drift on long same-process sessions).

This script is a thin wrapper: it imports mimo.py and reuses its
mmw_set_config()/mmw_init()/run_one_capture()/IR-sensor machinery directly
(mimo.config_dict is set here, same as mimo.py's main() would) so there is
exactly one implementation of the actual capture logic to keep correct.

Examples:
    python3 mimo_robotic.py --frames 2000 --radar-config cascade_tx5_rx16_robotic
    python3 mimo_robotic.py --frames 2000 --distrobox ros2 --motion-dir ~/Projects/ur_ws
    # exp_name> test motion_line_horz
    # exp_name> test circle
"""
import os
import sys
import copy
import glob
import shlex
import signal
import argparse
import subprocess
import time
from datetime import datetime

import mimo
import mmwcas
from utility import signal_handler, TeeLogger, upload_files_to_tda
from radar_config import RADAR_CONFIGS, get_radar_config

DEFAULT_RADAR_CONFIG = "cascade_tx5_rx16_robotic"
DEFAULT_DISTROBOX = "ros2"
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_UR_WS = os.path.join(_HERE, "ur_ws")
DEFAULT_MOTION_DIR = (
    _REPO_UR_WS if os.path.isdir(_REPO_UR_WS)
    else os.path.expanduser("~/Projects/ur_ws")
)
TRAJ_DIR = "robot_trajectories"


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


def _motion_short_name(filename):
    """motion_circle.py -> circle; other .py stems unchanged."""
    stem = filename[:-3] if filename.endswith(".py") else filename
    if stem.startswith("motion_"):
        return stem[len("motion_"):]
    return stem


def resolve_motion_script(name, motion_dir):
    """
    Resolve a prompt token to a motion_*.py path.

    Accepts: circle, motion_circle, motion_circle.py, or an absolute path.
    """
    raw = name
    candidates = []
    if os.path.isabs(raw):
        candidates.append(raw)
    else:
        base = os.path.basename(raw)
        if not base.endswith(".py"):
            base_py = base + ".py"
        else:
            base_py = base
        candidates.append(os.path.join(motion_dir, base_py))
        if not base_py.startswith("motion_"):
            candidates.append(os.path.join(motion_dir, "motion_" + base_py))
        if base_py.startswith("motion_"):
            candidates.append(os.path.join(motion_dir, base_py[len("motion_"):]))

    allowed = {os.path.abspath(os.path.join(motion_dir, s))
               for s in _list_motion_scripts(motion_dir)}
    for path in candidates:
        resolved = os.path.abspath(path)
        if resolved in allowed and os.path.isfile(resolved):
            return resolved

    available = ", ".join(_motion_short_name(s)
                          for s in _list_motion_scripts(motion_dir))
    raise FileNotFoundError(
        f"Unknown motion '{name}'. Available: {available}"
    )


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


def warmup_distrobox(distrobox_name):
    """Start the container now so the later enter is not paid at frame start."""
    cmd = distrobox_cmd(distrobox_name, ["true"])
    print(f"[ROBOT] warming distrobox '{distrobox_name}' "
          f"({shlex.join(cmd)}) ...")
    try:
        result = subprocess.run(cmd, check=False)
    except FileNotFoundError:
        print("Error: distrobox not found on PATH. Install distrobox, or this "
              "host is not the machine that owns the UR workspace.")
        sys.exit(1)
    if result.returncode != 0:
        print(f"Error: distrobox enter '{distrobox_name}' failed "
              f"(exit {result.returncode}). Check `distrobox list`.")
        sys.exit(1)
    print(f"[ROBOT] distrobox '{distrobox_name}' is ready.")


class MotionJob:
    """Spawn motion_traj.py at frame start; wait() then upload the .npy."""

    def __init__(self, distrobox_name, inner_argv, motion_dir, tda_ip):
        self.distrobox_name = distrobox_name
        self.inner_argv = inner_argv
        self.motion_dir = motion_dir
        self.tda_ip = tda_ip
        self.proc = None
        self.traj_path = None

    def start(self, capture_dir=None):
        name = capture_dir or datetime.now().strftime("traj_%y%m%d_%H%M%S")
        os.makedirs(TRAJ_DIR, exist_ok=True)
        self.traj_path = os.path.abspath(
            os.path.join(TRAJ_DIR, f"{name}_pos-timestamps.npy"))
        try:
            wrapped = wrap_motion_traj(
                self.inner_argv, self.motion_dir, self.traj_path)
        except FileNotFoundError as e:
            print(f"[ROBOT] {e}; launching motion without trajectory recording.")
            wrapped = self.inner_argv
            self.traj_path = None
        self.cmd = distrobox_cmd(self.distrobox_name, wrapped)
        print(f"[ROBOT] frames started - launching: {shlex.join(self.cmd)}")
        try:
            # Own session: Ctrl+C on the radar REPL must not SIGINT the arm.
            self.proc = subprocess.Popen(
                self.cmd,
                cwd=self.motion_dir,
                start_new_session=True,
            )
        except OSError as e:
            print(f"[ROBOT] failed to launch motion ({e}); capture continues.")
            self.proc = None

    def wait_until_done(self, fallback_s):
        """Block until the motion process exits, then let the radar stop.

        fallback_s is used only if the motion process never started (then
        we wait the --frames window so the capture is not zero-length).
        """
        if self.proc is None:
            print("[ROBOT] motion did not start; waiting the --frames window.")
            time.sleep(fallback_s)
            return
        print("[ROBOT] capturing until motion ends...")
        rc = self.proc.wait()
        if rc == 0:
            print("[ROBOT] motion finished - stopping radar.")
        else:
            print(f"[ROBOT] motion exited {rc} - stopping radar.")

    def wait(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        print("[ROBOT] waiting for motion to finish "
              "(Ctrl+C will not stop the arm)...")
        rc = self.proc.wait()
        if rc == 0:
            print("[ROBOT] motion finished.")
        else:
            print(f"[ROBOT] motion exited {rc}.")

    def upload_trajectory(self, capture_dir):
        if not capture_dir or not self.traj_path:
            return
        if not os.path.isfile(self.traj_path):
            print(f"[ROBOT] trajectory .npy not written ({self.traj_path}); "
                  "skipping TDA upload.")
            return
        print(f"[ROBOT] uploading trajectory {self.traj_path} to TDA "
              f"capture '{capture_dir}' ...")
        upload_files_to_tda([self.traj_path], capture_dir, self.tda_ip)


def _read_prompt(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def run_robotic(args):
    """
    REPL loop: configure once (RF numFrames=0, i.e. infinite framing), then
    repeatedly prompt for an exp_name and an optional motion command.
    If a motion is given, framing starts with the arm and stops when the
    motion process exits. --frames is the length of radar-only captures
    (no motion on the prompt), not a motion timeout.
    """
    period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    scripts = _list_motion_scripts(args.motion_dir)
    print("\n=== Robotic multi-capture mode ===")
    print("Radar is configured and the connection to the TDA is live.")
    print(f"Radar-only captures in this session last {args.frames} frames "
          f"(~{args.frames * period_ms / 1000.0:.1f}s @ {period_ms:.0f} ms/frame).")
    print("With a motion on the prompt, the radar STOPS when the arm motion "
          "finishes (not at --frames).")
    print(f"Motion runs inside distrobox '{args.distrobox}' from "
          f"{args.motion_dir} via motion_traj.py, started at "
          "mmw_start_frame() (not during configure/arm). TCP trajectory "
          f"is saved to {TRAJ_DIR}/<capture_dir>_pos-timestamps.npy and "
          "uploaded to the TDA capture directory.")
    print("At the prompt, type an exp_name, optionally followed by a motion:")
    print("  test  line_horz   # radar + arm  (also: motion_line_horz, motion_line_horz.py)")
    print("  test              # radar only, arm stays still")
    print("  (exp_name same role as mimo.py --exp-name; final TDA dir is "
          "exp_name_<YYMMDD>_<HHMMSS>)")
    if scripts:
        print("Available motions:")
        for name in scripts:
            print(f"  {_motion_short_name(name):<24}  ({name})")
    print(f"Frame period: {period_ms:.0f} ms  ({1000.0/period_ms:.1f} fps)")
    print("Type 'quit'/'exit' or leave blank to stop.\n")

    while True:
        line = _read_prompt("exp_name> ")
        if not line or line in ("quit", "exit"):
            break

        parts = line.split(maxsplit=1)
        exp_name = parts[0]
        motion_line = parts[1] if len(parts) > 1 else ""

        inner_argv = None
        if motion_line:
            try:
                inner_argv = resolve_motion_argv(motion_line, args.motion_dir)
            except (ValueError, FileNotFoundError) as e:
                print(f"[ROBOT] {e}")
                continue
        else:
            print("[ROBOT] no motion given - radar capture only, arm stays still.")

        tee = None
        if not args.no_log:
            provisional = os.path.join(
                "logs", f"mimo_robotic_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
            try:
                tee = TeeLogger(provisional).start()
            except (OSError, ImportError) as e:
                print(f"[LOG] terminal logging unavailable ({e}); continuing "
                      f"without it.")
                tee = None

        capture_dir = None
        capture_ok = False
        motion = None
        wait_fn = None
        if inner_argv is not None:
            motion = MotionJob(args.distrobox, inner_argv, args.motion_dir,
                               args.tda_ip)
            period_s = period_ms / 1000.0
            wait_s = mimo._wait_s_for_frames(args.frames, period_s)
            wait_fn = lambda t=wait_s: motion.wait_until_done(t)
        try:
            status, capture_dir = mimo.run_one_capture(
                exp_name, args.frames, args.tda_ip,
                on_frame_start=None if motion is None else motion.start,
                wait_fn=wait_fn)
            if status == 0:
                print(f">>> Capture '{capture_dir}' completed successfully.\n")
                capture_ok = True
            else:
                print(f">>> Capture '{capture_dir}' finished with errors "
                      f"(status {status}). Continuing...\n")
        finally:
            if motion is not None:
                motion.wait()
                motion.upload_trajectory(capture_dir)
            mimo._finish_log(tee, capture_dir, capture_ok, args.tda_ip)

    print("Exiting robotic mode.")


def main():
    parser = argparse.ArgumentParser(
        description='TIDEP-01012 MIMO Cascade Radar - robotic-arm REPL capture')
    parser.add_argument('-f', '--frames',
                        type=int,
                        default=100,
                        help='Frame count for radar-only captures (prompt is just '
                             'an exp_name, no motion). With a motion, the radar '
                             'runs until the arm finishes; --frames is not a stop '
                             'timer. Default: 100.')
    parser.add_argument('-c', '--radar-config',
                        type=str,
                        default=DEFAULT_RADAR_CONFIG,
                        choices=sorted(RADAR_CONFIGS),
                        help='Named RF/geometry preset from radar_configs/*.toml '
                             f"(default: '{DEFAULT_RADAR_CONFIG}').")
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
    parser.add_argument('--no-log',
                        action='store_true',
                        help='Do not record terminal output per capture. By '
                             'default every capture\'s printed output (including '
                             'the C-level STATUS lines from mmwcas) is written to '
                             'logs/<capture_dir>.log and uploaded into that '
                             'capture\'s TDA directory alongside the .bin data, '
                             'same as mimo.py.')
    parser.add_argument('--distrobox',
                        type=str,
                        default=DEFAULT_DISTROBOX,
                        help='Distrobox container that owns the UR RTDE env '
                             f"(default: '{DEFAULT_DISTROBOX}').")
    parser.add_argument('--motion-dir',
                        type=str,
                        default=DEFAULT_MOTION_DIR,
                        help='Directory of motion_*.py scripts '
                             f"(default: '{DEFAULT_MOTION_DIR}').")

    args = parser.parse_args()
    args.motion_dir = os.path.abspath(os.path.expanduser(args.motion_dir))

    signal.signal(signal.SIGINT, signal_handler)

    if args.frames < 1:
        print("Error: --frames must be >= 1")
        sys.exit(1)
    if args.radar_config not in RADAR_CONFIGS:
        print(f"Error: unknown --radar-config '{args.radar_config}'")
        sys.exit(1)

    # mimo.run_one_capture() reads mimo.config_dict as a module-level global -
    # set it here exactly like mimo.py's main() does, so mimo.py's capture
    # logic stays the single source of truth.
    mimo.config_dict = copy.deepcopy(get_radar_config(args.radar_config))
    period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    approx_s = args.frames * period_ms / 1000.0

    # RF chips stay at numFrames=0 (infinite) - every capture's length is
    # controlled by how long we wait before mmw_stop_frame() (TDA arming),
    # not by RF reconfiguration. That wait is the same fixed args.frames for
    # the whole session (see run_robotic()'s docstring for why).
    mimo.config_dict["mimo"]["frame"]["numFrames"] = 0

    print(f"Capture frames   : {args.frames}  (~{approx_s:.1f}s @ {period_ms:.0f} ms/frame)"
          "  [radar-only length; with a motion, radar follows the arm]")
    print(f"Radar config     : {args.radar_config}")
    print(f"Distrobox        : {args.distrobox}")
    print(f"Motion dir       : {args.motion_dir}")

    status = mmwcas.mmw_set_config(mimo.config_dict)
    if status != 0:
        print(f"Configuration error: {status}")
        raise ValueError(f"mmw_set_config failed with status {status}")

    status = mmwcas.mmw_init()
    assert status == 0, ValueError("mmw_init failed")

    time.sleep(2)
    os.makedirs("mmwave_json_files", exist_ok=True)

    warmup_distrobox(args.distrobox)

    if not args.no_ir:
        mimo.setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
    else:
        print("[IR] IR sensor timestamp logging disabled (--no-ir).")

    try:
        run_robotic(args)
    except KeyboardInterrupt:
        print("\n\nRobotic mode interrupted by user")
        sys.exit(130)
    finally:
        mimo.teardown_ir_sensor()


if __name__ == "__main__":
    main()
