"""
mimo_interactive.py - DEPRECATED interactive REPL for mimo.py's radar capture
(TIDEP-01012 MIMO Cascade Radar).

*** DEPRECATED (2026-07-22) *** - kept only for ad-hoc manual testing at the
bench. Do NOT use for real data collection / bridge deployment. Two known
hardware issues, neither fixable from the host side:
  1. Frame counts here are wall-clock-based, not hardware-exact (the RF
     chips are configured once at startup with numFrames=0/infinite; each
     prompt's frame count only controls how long we wait before
     mmw_stop_frame()). A live per-capture RF reconfigure was tried and
     reverted after it wedged the RF chips on real hardware.
  2. Repeated captures within one long-lived process are exactly the
     scenario that showed SSD/network throughput drift (capture sizes
     shrinking run to run) - see mimo.py's docstring. This script is
     structurally the worst case for that (a single process making an
     arbitrary number of back-to-back captures).

For real captures, use `mimo.py` (single exact capture per process
invocation) driven by an external shell loop instead.

This script is a thin wrapper: it imports mimo.py and reuses its
mmw_set_config()/mmw_init()/run_one_capture()/IR-sensor machinery directly
(mimo.config_dict is set here, same as mimo.py's main() would) so there is
exactly one implementation of the actual capture logic to keep correct.
"""
import sys
import copy
import signal
import argparse

import mimo
import mmwcas
from utility import signal_handler
from radar_config import RADAR_CONFIGS, DEFAULT_RADAR_CONFIG, get_radar_config


def run_interactive(args):
    """
    REPL loop: configure once (RF numFrames=0, i.e. infinite framing), then
    repeatedly prompt for an experiment name and optional frame count. Each
    capture arms the TDA and waits num_frames x framePeriodicity (+ margin)
    before calling mmw_stop_frame() - no RF reconfiguration between captures
    (see mimo.run_one_capture()'s docstring for why). Frame counts here are
    therefore approximate (wall-clock based, a few % jitter run to run), not
    hardware-exact.
    """
    period_ms = float(mimo.config_dict["mimo"]["frame"]["framePeriodicity"])
    print("\n=== Interactive multi-capture mode [DEPRECATED] ===")
    print("Radar is configured and the connection to the TDA is live.")
    print("At the prompt, type an experiment name to arm + record, e.g.:")
    print(f"  bridge_test          (uses --frames default: {args.frames})")
    print(f"  bridge_test 300      (300 frames for this capture only)")
    print("Frame counts are wall-clock based (approximate, +/- a few % run to "
          "run) - the RF chips are NOT reconfigured per prompt.")
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
        try:
            num_frames = int(parts[1]) if len(parts) >= 2 else args.frames
        except ValueError:
            print(f"Could not parse frame count; using default {args.frames}.")
            num_frames = args.frames
        if num_frames < 1:
            print("Frame count must be >= 1; using configured --frames.")
            num_frames = args.frames

        status, capture_dir = mimo.run_one_capture(exp_name, num_frames, args.tda_ip)
        if status == 0:
            print(f">>> Capture '{capture_dir}' completed successfully.\n")
        else:
            print(f">>> Capture '{capture_dir}' finished with errors (status {status}). Continuing...\n")

    print("Exiting interactive mode.")


def main():
    print("*** mimo_interactive.py is DEPRECATED - bench/manual testing only. ***")
    print("*** For real captures use mimo.py (via a shell loop). ***\n")

    parser = argparse.ArgumentParser(
        description='[DEPRECATED] TIDEP-01012 MIMO Cascade Radar - interactive REPL capture')
    parser.add_argument('--frames',
                        type=int,
                        default=100,
                        help='Default frame count per capture (override per-prompt: '
                             '"<name> <frames>"). Default: 100.')
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

    # RF chips stay at numFrames=0 (infinite) so each prompt can request a
    # different length via TDA arming rather than RF reconfiguration.
    mimo.config_dict["mimo"]["frame"]["numFrames"] = 0

    print(f"Capture frames   : {args.frames}  (~{approx_s:.1f}s @ {period_ms:.0f} ms/frame)"
          "  [default; override per prompt]")
    print(f"Radar config     : {args.radar_config}")

    status = mmwcas.mmw_set_config(mimo.config_dict)
    if status != 0:
        print(f"Configuration error: {status}")
        raise ValueError(f"mmw_set_config failed with status {status}")

    status = mmwcas.mmw_init()
    assert status == 0, ValueError("mmw_init failed")

    import os
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
