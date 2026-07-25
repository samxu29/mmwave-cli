"""
mimo_interactive.py - interactive REPL for mimo.py's radar capture
(TIDEP-01012 MIMO Cascade Radar).

Configure the RF chips once, then repeat many named captures without
re-launching the process. The frame count is fixed for the whole session -
set once via --frames at startup, applied to every capture - rather than
overridable per prompt. Each prompt takes only an experiment name. This
keeps every capture in the session identical in length (same TDA
pre-allocation sizing, same arm/wait/stop timing), which is what varying
per-prompt frame counts put at risk: mismatched pre-allocation and wait
windows are a contributor to dropped frames, so removing that source of
variation lowers drop chances across a session.

Two known hardware constraints remain, neither fixable from the host side:
  1. Frame counts here are wall-clock-based, not hardware-exact (the RF
     chips are configured once at startup with numFrames=0/infinite; the
     fixed frame count only controls how long we wait before
     mmw_stop_frame()). A live per-capture RF reconfigure was tried and
     reverted after it wedged the RF chips on real hardware.
  2. Repeated captures within one long-lived process are exactly the
     scenario that showed SSD/network throughput drift (capture sizes
     shrinking run to run) - see mimo.py's docstring. This script is
     structurally the worst case for that (a single process making an
     arbitrary number of back-to-back captures) - keep sessions short and
     watch for drift on long runs.

This script is a thin wrapper: it imports mimo.py and reuses its
mmw_set_config()/mmw_init()/run_one_capture()/IR-sensor machinery directly
(mimo.config_dict is set here, same as mimo.py's main() would) so there is
exactly one implementation of the actual capture logic to keep correct.
"""
import os
import sys
import copy
import signal
import argparse
from datetime import datetime

import mimo
import mmwcas
from utility import signal_handler, TeeLogger
from radar_config import RADAR_CONFIGS, DEFAULT_RADAR_CONFIG, get_radar_config


def run_interactive(args):
    """
    REPL loop: configure once (RF numFrames=0, i.e. infinite framing), then
    repeatedly prompt for an experiment name only. Every capture in the
    session uses the same fixed args.frames count, set once at startup -
    each capture arms the TDA and waits args.frames x framePeriodicity
    (+ margin) before calling mmw_stop_frame() - no RF reconfiguration
    between captures (see mimo.run_one_capture()'s docstring for why), and
    no per-prompt frame-count override, so TDA pre-allocation sizing and
    wait timing stay identical capture to capture. Frame counts are
    therefore approximate (wall-clock based, a few % jitter run to run), not
    hardware-exact.
    """
    period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    print("\n=== Interactive multi-capture mode ===")
    print("Radar is configured and the connection to the TDA is live.")
    print(f"Every capture in this session uses {args.frames} frames "
          f"(~{args.frames * period_ms / 1000.0:.1f}s @ {period_ms:.0f} ms/frame) - "
          "fixed at startup via --frames.")
    print("At the prompt, type an experiment name to arm + record, e.g.:")
    print("  bridge_test")
    print(f"Frame period: {period_ms:.0f} ms  ({1000.0/period_ms:.1f} fps)")
    print("Type 'quit'/'exit' or leave blank to stop.\n")

    while True:
        try:
            line = input("experiment> ").strip()
        except EOFError:
            break
        if not line or line in ("quit", "exit"):
            break

        parts = line.split()
        exp_name = parts[0]
        if len(parts) > 1:
            print(f"Frame count is fixed for this session ({args.frames}); "
                  f"ignoring extra input {parts[1:]}.")

        # One TeeLogger session per capture (not one for the whole REPL
        # session) - mimo.py's _finish_log() renames the log after the
        # capture_dir it just recorded and uploads it into that same TDA
        # capture directory, which only makes sense 1:1 per capture.
        tee = None
        if not args.no_log:
            provisional = os.path.join(
                "logs", f"mimo_interactive_{datetime.now().strftime('%y%m%d_%H%M%S')}.log")
            try:
                tee = TeeLogger(provisional).start()
            except (OSError, ImportError) as e:
                print(f"[LOG] terminal logging unavailable ({e}); continuing "
                      f"without it.")
                tee = None

        capture_dir = None
        capture_ok = False
        try:
            status, capture_dir = mimo.run_one_capture(exp_name, args.frames, args.tda_ip)
            if status == 0:
                print(f">>> Capture '{capture_dir}' completed successfully.\n")
                capture_ok = True
            else:
                print(f">>> Capture '{capture_dir}' finished with errors (status {status}). Continuing...\n")
        finally:
            mimo._finish_log(tee, capture_dir, capture_ok, args.tda_ip)

    print("Exiting interactive mode.")


def main():
    parser = argparse.ArgumentParser(
        description='TIDEP-01012 MIMO Cascade Radar - interactive REPL capture')
    parser.add_argument('--frames',
                        type=int,
                        default=100,
                        help='Frame count for every capture in this session, fixed at '
                             'startup (no per-prompt override). Default: 100.')
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
    parser.add_argument('--radar-config',
                        type=str,
                        default=DEFAULT_RADAR_CONFIG,
                        choices=sorted(RADAR_CONFIGS),
                        help='Named RF/geometry preset from radar_configs/*.toml '
                             f"(default: '{DEFAULT_RADAR_CONFIG}').")
    parser.add_argument('--no-log',
                        action='store_true',
                        help='Do not record terminal output per capture. By '
                             'default every capture\'s printed output (including '
                             'the C-level STATUS lines from mmwcas) is written to '
                             'logs/<capture_dir>.log and uploaded into that '
                             'capture\'s TDA directory alongside the .bin data, '
                             'same as mimo.py.')

    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    if args.frames < 1:
        print("Error: --frames must be >= 1")
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
    # the whole session (see run_interactive()'s docstring for why).
    mimo.config_dict["mimo"]["frame"]["numFrames"] = 0

    print(f"Capture frames   : {args.frames}  (~{approx_s:.1f}s @ {period_ms:.0f} ms/frame)"
          "  [fixed for this session]")
    print(f"Radar config     : {args.radar_config}")

    status = mmwcas.mmw_set_config(mimo.config_dict)
    if status != 0:
        print(f"Configuration error: {status}")
        raise ValueError(f"mmw_set_config failed with status {status}")

    status = mmwcas.mmw_init()
    assert status == 0, ValueError("mmw_init failed")

    import time
    time.sleep(2)
    os.makedirs("mmwave_json_files", exist_ok=True)

    if not args.no_ir:
        mimo.setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
    else:
        print("[IR] IR sensor timestamp logging disabled (--no-ir).")

    try:
        run_interactive(args)
    except KeyboardInterrupt:
        print("\n\nInteractive mode interrupted by user")
        sys.exit(130)
    finally:
        mimo.teardown_ir_sensor()


if __name__ == "__main__":
    main()
