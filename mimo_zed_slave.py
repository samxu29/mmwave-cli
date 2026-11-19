"""
mimo_zed_slave.py - cascade radar capture service driven by the ZED/Record3D host
(zed_record3d/capture_zed_record3d_cascade.py and its helios variants).

Same capture path as mimo_interactive.py (configure the RF chips once, then
mimo.run_one_capture() per take), but built to be controlled by a program
instead of a person:

  * Commands arrive on stdin, one per line:
        capture <exp_name> [<YYMMDD_HHMMSS>]
                             record one take into TDA dir <exp_name>_<YYMMDD_HHMMSS>;
                             the host sends its take timestamp so the camera and TDA
                             folders share one name (default: this Pi's local time)
        ping                 answer with a "pong" event
        exit                 tear down and exit (also: quit, or EOF on stdin)
  * Results go out as machine-readable events, one JSON object per line,
    prefixed with "@MZS ". They are written to a private copy of the
    original stdout taken at startup, so they bypass TeeLogger's pty/tee
    chain (no extra latency on the "capturing" time anchor) and stay out of
    the per-capture log. Everything else (mimo.py prints, mmwcas STATUS
    lines) is ordinary human-readable output and is still logged/uploaded
    exactly like mimo_interactive.py does. Each event starts with a newline
    so it never glues onto a half-written log line; readers should look for
    "@MZS " anywhere in a line.

Events (all carry "event"; "t" is this Pi's time.time()):
    hello      {version, config, frames, pid}         process started, RF setup begins
    ready      {config, frames, frame_period_ms, wait_s}
                                                     RF configured, waiting for a command
    capturing  {capture_dir, frames, wait_s}          framing confirmed (mmw_start_frame()
                                                     returned 0) - the host's time anchor
    done       {ok, status, stage, capture_dir, frames_idx, frame_check, files, log,
                ir, needs_restart, message}           take finished (ok or not)
               frame_check: {frames, requested, gaps, skipped, largest_gap_ms,
                first_gap_frame} from the TDA idx.bin timestamps (null if unavailable)
    pong, error {message}, bye {code}

Failure handling (the reasons this script exists):
  * mimo.run_one_capture() returns early on an arm/start/stop/de-arm error
    without stopping the RF chips or de-arming the TDA. The chips run with
    numFrames=0 (infinite), so a failed start used to leave the radar
    streaming into the TDA until /mnt/ssd filled up. Here every failed take
    is followed by mmw_stop_frame() + mmw_dearming_tda().
  * A take whose TDA directory never appears is reported as a failure
    (stage "verify"), not "completed successfully".
  * After any failed take the RF chips are often wedged (every later
    mmw_start_frame() fails until a fresh mmw_init()), and mmw_init() is not
    safe to repeat inside one process. So the done event says
    needs_restart=true and this process exits; the host starts a new one,
    which powers up and reconfigures the chips from scratch.
  * SIGHUP/SIGTERM (host or SSH gone mid-take) stop and de-arm the radar
    before exiting instead of leaving it framing.

Run by hand for testing (type the commands):
    python3 -u mimo_zed_slave.py -c cascade_tx7_rx16_5rps_ns128 -f 300
    capture test
    exit
"""
import argparse
import copy
import json
import os
import re
import signal
import sys
import time
from datetime import datetime

import numpy as np

try:
    import readline  # noqa: F401  (line editing when run by hand)
except ImportError:
    pass

import mimo
import mmwcas
from utility import TeeLogger
from radar_config import RADAR_CONFIGS, DEFAULT_RADAR_CONFIG, get_radar_config

PROTOCOL_VERSION = 2
EVENT_PREFIX = "@MZS "
EXIT_NEEDS_RESTART = 3
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
_STAMP_RE = re.compile(r"^\d{6}_\d{6}$")


class Hangup(Exception):
    """Raised from the SIGHUP/SIGTERM handler to unwind a take and clean up."""


class Events:
    """Writes @MZS events to a duplicate of the stdout fd this process started with."""

    def __init__(self):
        sys.stdout.flush()
        self._fd = os.dup(1)
        self.closed = False

    def send(self, event, **fields):
        if self.closed:
            return
        fields = {"event": event, "t": time.time(), **fields}
        line = "\n" + EVENT_PREFIX + json.dumps(fields, separators=(",", ":")) + "\n"
        try:
            os.write(self._fd, line.encode())
        except OSError:
            self.closed = True  # host gone; keep cleaning up anyway


class RadarCalls:
    """Proxy installed as mimo.mmwcas so every mmw_*() status run_one_capture() sees is recorded."""

    def __init__(self, real):
        self._real = real
        self.calls = []

    def __getattr__(self, name):
        attr = getattr(self._real, name)
        if not (name.startswith("mmw_") and callable(attr)):
            return attr

        def call(*args, **kwargs):
            status = attr(*args, **kwargs)
            self.calls.append((name, status))
            return status

        return call

    def status_of(self, name):
        """Last status returned by mmw_<name>(), or None if it was never called."""
        for called, status in reversed(self.calls):
            if called == name:
                return status
        return None


def _frames_from_files(files):
    """{file: frames} from the TDA's <dev>_0000_idx.bin sizes (one entry per captured frame)."""
    frames = {}
    for name, size in files:
        if name.endswith("_idx.bin") and str(size).isdigit():
            count = mimo._frames_from_idx_size(int(size))
            if count is not None:
                frames[name] = count
    return frames


def _frame_check(capture_dir, period_ms, requested):
    """Skipped-frame summary from the TDA idx.bin timestamps run_one_capture() saved, or None.

    A gap is an interval above 1.5x the configured frame period; round(gap/period) - 1
    frames are counted as skipped in it (the same heuristic as check_timestamp.py).
    """
    path = os.path.join("rf_frame_timestamps", f"{capture_dir}_rf_frame_timestamps.npy")
    if not capture_dir or not os.path.exists(path):
        return None
    try:
        ts = np.asarray(np.load(path), dtype=float)
    except Exception as exc:
        print(f"[slave] could not read {path} ({exc!r})")
        return None
    period_s = period_ms / 1000.0
    dt = np.diff(ts)
    gap_at = np.flatnonzero(dt > 1.5 * period_s)
    return {
        "frames": int(ts.size),
        "requested": int(requested),
        "gaps": int(gap_at.size),
        "skipped": int(sum(round(dt[i] / period_s) - 1 for i in gap_at)),
        "largest_gap_ms": round(float(dt.max()) * 1000.0, 1) if dt.size else None,
        "first_gap_frame": int(gap_at[0]) + 1 if gap_at.size else None,
    }


def _stop_radar():
    """Best effort: stop framing on every device and de-arm the TDA. Never raises."""
    for step in (mmwcas.mmw_stop_frame, mmwcas.mmw_dearming_tda):
        try:
            step()
        except Exception as exc:  # cleanup must reach the next step
            print(f"[slave] {step.__name__} during cleanup raised {exc!r}")


def run_take(exp_name, stamp, args, events):
    """Record one take. Returns True when this process must exit (radar needs a fresh init)."""
    radar = RadarCalls(mmwcas)
    checked = {"files": None}
    original_check = mimo.check_captured_files

    def check_captured_files(capture_dir, tda_ip, *rest, **kwargs):
        result = original_check(capture_dir, tda_ip, *rest, **kwargs)
        checked["files"] = result
        return result

    def on_frame_start(capture_dir):
        events.send("capturing", capture_dir=capture_dir, frames=args.frames, wait_s=args.wait_s)

    tee = None
    if not args.no_log:
        provisional = os.path.join(
            "logs", f"mimo_zed_slave_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
        try:
            tee = TeeLogger(provisional).start()
        except (OSError, ImportError) as exc:
            print(f"[LOG] terminal logging unavailable ({exc}); continuing without it.")

    status, capture_dir, message, hangup = None, None, "", False
    mimo.mmwcas = radar
    mimo.check_captured_files = check_captured_files
    try:
        status, capture_dir = mimo.run_one_capture(
            exp_name, args.frames, args.tda_ip, on_frame_start=on_frame_start, timestamp=stamp)
    except Hangup:
        hangup, message = True, "host disconnected during the take"
    except Exception as exc:
        message = f"run_one_capture raised {exc!r}"
    finally:
        mimo.mmwcas = mmwcas
        mimo.check_captured_files = original_check

    started = radar.status_of("mmw_start_frame") == 0
    files = (checked["files"] or (False, 0, []))[2]
    if message:
        stage = "exception"
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
        message = f"mmw_{stage} returned status {status}" if stage != "ok" else f"status {status}"

    if not ok:
        print(f"[slave] take failed at stage '{stage}': {message}. Stopping the radar and de-arming the TDA.")
        _stop_radar()
    if capture_dir is None:
        capture_dir = ""

    print(f">>> Capture '{capture_dir}' " + ("completed successfully." if ok else f"FAILED ({stage}): {message}"))
    log_path = None
    try:
        mimo._finish_log(tee, capture_dir or None, ok, args.tda_ip)
        if tee is not None and capture_dir:
            log_path = os.path.join("logs", f"{capture_dir}.log")
    except Exception as exc:
        print(f"[LOG] finishing the log failed ({exc!r})")

    ir_path = os.path.join("ir_timestamps", f"{capture_dir}_ir_timestamps.npy")
    events.send(
        "done",
        ok=ok,
        status=status,
        stage=stage,
        capture_dir=capture_dir,
        frames_idx=_frames_from_files(files),
        frame_check=_frame_check(capture_dir, args.period_ms, args.frames) if ok else None,
        files=len(files),
        log=log_path,
        ir=ir_path if (started and capture_dir and os.path.exists(ir_path)) else None,
        needs_restart=not ok,
        message=message,
    )
    return hangup or not ok


def serve(args, events):
    events.send("ready", config=args.radar_config, frames=args.frames,
                frame_period_ms=args.period_ms, wait_s=args.wait_s)
    while True:
        try:
            line = sys.stdin.readline()
        except Hangup:
            return 0
        if not line:
            return 0
        parts = line.split()
        if not parts:
            continue
        command = parts[0].lower()
        if command in ("exit", "quit"):
            return 0
        if command == "ping":
            events.send("pong")
        elif (command == "capture" and len(parts) in (2, 3) and _NAME_RE.match(parts[1])
              and (len(parts) == 2 or _STAMP_RE.match(parts[2]))):
            try:
                needs_restart = run_take(parts[1], parts[2] if len(parts) == 3 else None, args, events)
            except Hangup:
                _stop_radar()
                return 0
            if needs_restart:
                return EXIT_NEEDS_RESTART
            events.send("ready", config=args.radar_config, frames=args.frames,
                        frame_period_ms=args.period_ms, wait_s=args.wait_s)
        else:
            events.send("error", message=f"unknown or malformed command: {line.strip()!r}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-f", "--frames", type=int, default=300,
                        help="Frames per take, fixed for the session (default: 300).")
    parser.add_argument("-c", "--radar-config", default=DEFAULT_RADAR_CONFIG, choices=sorted(RADAR_CONFIGS),
                        help=f"Preset from radar_configs/*.toml (default: {DEFAULT_RADAR_CONFIG}).")
    parser.add_argument("--tda-ip", default="192.168.33.180", help="TDA board IP (default: 192.168.33.180).")
    parser.add_argument("--no-ir", action="store_true", help="Disable IR sensor timestamp logging.")
    parser.add_argument("--ir-pin", type=int, default=4, help="BCM GPIO pin of the IR sensor (default: 4).")
    parser.add_argument("--ir-bounce-ms", type=int, default=200, help="IR debounce in ms (default: 200).")
    parser.add_argument("--no-log", action="store_true",
                        help="Do not write/upload logs/<capture_dir>.log per take.")
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be >= 1")

    events = Events()
    hung_up = []

    def on_signal(signum, _frame):
        if not hung_up:  # once only, so a second signal cannot abort the stop/de-arm cleanup
            hung_up.append(signum)
            raise Hangup(signal.Signals(signum).name)

    signal.signal(signal.SIGHUP, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # stop with "exit" / SIGTERM, not a stray Ctrl+C

    mimo.config_dict = copy.deepcopy(get_radar_config(args.radar_config))
    mimo.config_dict["mimo"]["frame"]["numFrames"] = 0  # length is set by the host-side wait, see mimo_interactive.py
    args.period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    args.wait_s = mimo._wait_s_for_frames(args.frames, mimo._frame_period_s(mimo.config_dict))
    events.send("hello", version=PROTOCOL_VERSION, config=args.radar_config, frames=args.frames, pid=os.getpid())
    print(f"Capture frames   : {args.frames}  (~{args.frames * args.period_ms / 1000:.1f}s "
          f"@ {args.period_ms:.0f} ms/frame)  [fixed for this session]")
    print(f"Radar config     : {args.radar_config}")

    code = 1
    try:
        status = mmwcas.mmw_set_config(mimo.config_dict)
        if status != 0:
            raise RuntimeError(f"mmw_set_config failed with status {status}")
        status = mmwcas.mmw_init(args.tda_ip)
        if status != 0:
            raise RuntimeError(f"mmw_init failed with status {status}")
        time.sleep(2)
        os.makedirs("mmwave_json_files", exist_ok=True)
        if args.no_ir:
            print("[IR] IR sensor timestamp logging disabled (--no-ir).")
        else:
            mimo.setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
        code = serve(args, events)
    except Hangup:
        code = 0
    except Exception as exc:
        print(f"[slave] fatal: {exc!r}")
        events.send("error", message=repr(exc))
    finally:
        mimo.teardown_ir_sensor()
        events.send("bye", code=code)
    return code


if __name__ == "__main__":
    sys.exit(main())
