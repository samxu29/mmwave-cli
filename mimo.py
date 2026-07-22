"""
mimo.py - TIDEP-01012 MIMO Cascade Radar Control (Python / mmwcas Cython wrapper)

RF/geometry configuration (chirp profiles, idle times, antenna geometry,
frame timing) has moved OUT of this file and into radar_config.py
(2026-07-22) - see that module's header for details and for how to add a new
idle-time scheme or antenna geometry preset without editing this file. Select
a non-default preset with --radar-config <name>.

Two capture modes, both configure the radar ONCE then loop:
  - Default: automatic loop, driven by --num-loops/--inter-loop-time (used by
    pipeline.py).
  - --interactive: REPL loop mirroring mimo.c's --interactive - type an
    experiment name (+ optional seconds) at the `experiment>` prompt to arm
    + record; blank/quit/exit to stop. No reconfiguration between captures.

IR sensor timestamp logging (replaces the old run_experiment.sh + ir_logger.py
signal-based glue) is now built in: GPIO edge detection is armed once at
startup (same BCM pin/RISING-edge/200ms-debounce convention as ir_logger.py /
receiver_ir.py), and timestamp collection is gated on/off directly around
each capture's TDA framing window in run_one_capture() - no subprocess, no
signals. Output: ir_timestamps/<capture_dir>_ir_timestamps.npy, a float64
array of Unix epoch seconds (same convention as receiver_ir.py's
gpio_timestamps.npy). Silently disabled (with a printed notice) if
RPi.GPIO isn't importable or the pin can't be armed (e.g. running off-Pi),
or if --no-ir is passed.

After a successful capture, the local IR timestamps .npy and the .mmwave.json
config file are both SCP'd up into /mnt/ssd/<capture_dir>/ on the TDA board
itself (alongside the raw .bin data), so pipeline.py's existing
SCP-download-then-delete step picks them up together with the capture
automatically - no separate correlation step needed downstream.
"""
import time
import argparse
from datetime import datetime
import sys
import mmwcas
import signal
import numpy as np
from utility import export_config_to_json
from utility import check_captured_files
from utility import signal_handler
from utility import upload_files_to_tda
from radar_config import RADAR_CONFIGS, DEFAULT_RADAR_CONFIG, get_radar_config
import os

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

# RF/geometry config dict - selected from radar_config.RADAR_CONFIGS in
# main() based on --radar-config (defaults to cascade_tx3_rx16). Set here so
# run_one_capture() (which reads this module-level global) works whether
# called from main()'s automatic loop or run_interactive().
config_dict = get_radar_config(DEFAULT_RADAR_CONFIG)

# Global flag for graceful shutdown
shutdown_flag = False

# IR sensor state - populated by setup_ir_sensor(), consumed by
# run_one_capture(). ir_recording gates whether the callback (which fires on
# every rising edge for as long as GPIO edge detection is armed) actually
# records anything, so edges outside a capture window are discarded, never
# leaking into the wrong experiment's file.
ir_enabled = False
ir_recording = False
ir_timestamps = []


def _ir_sensor_callback(channel):
    if ir_recording:
        ir_timestamps.append(time.time())


def setup_ir_sensor(pin, bounce_ms):
    """
    Arm GPIO rising-edge detection on the IR sensor pin (same BCM pin/pull-up/
    RISING-edge convention as ir_logger.py and receiver_ir.py's TCRT5000
    wiring). Detection stays armed for the whole process lifetime; actual
    timestamp collection is toggled per-capture via the ir_recording flag.
    """
    global ir_enabled
    if not _HAS_GPIO:
        print("[IR] RPi.GPIO not available - IR sensor timestamp logging disabled.")
        return
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=_ir_sensor_callback, bouncetime=bounce_ms)
        ir_enabled = True
        print(f"[IR] Watching GPIO pin {pin} for rising edges (debounce {bounce_ms}ms).")
    except Exception as e:
        print(f"[IR] WARNING: failed to arm GPIO pin {pin} ({e}) - IR sensor timestamp logging disabled.")


def teardown_ir_sensor():
    if ir_enabled and _HAS_GPIO:
        GPIO.cleanup()


def save_ir_timestamps(capture_dir):
    """Write out whatever IR timestamps were collected for this capture's
    framing window and reset the buffer. Saved as a float64 .npy array
    (Unix epoch seconds), same convention as receiver_ir.py's
    gpio_timestamps.npy. Safe to call even if IR logging is disabled
    (no-op)."""
    global ir_timestamps
    if not ir_enabled:
        return None
    os.makedirs("ir_timestamps", exist_ok=True)
    path = os.path.join("ir_timestamps", f"{capture_dir}_ir_timestamps.npy")
    np.save(path, np.array(ir_timestamps, dtype=np.float64))
    print(f"[IR] Saved {len(ir_timestamps)} marker(s) to {path}")
    ir_timestamps = []
    return path


def run_one_capture(exp_label, duration_s, tda_ip):
    """
    Arm the TDA, record one capture, de-arm, then verify + export the
    .mmwave.json sidecar. Does NOT touch the RF chip configuration (profiles/
    chirps/frame geometry) - only the per-capture arm/trigger/stop sequence,
    same division of responsibility as mimo.c's run_capture(). Safe to call
    repeatedly after a single mmw_set_config()/mmw_init() (see --interactive).

    Returns (status, capture_dir) - status is 0 on full success.
    """
    timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
    capture_dir = f"{exp_label}_{timestamp}"

    print(f"\n>>> Capturing '{capture_dir}' for {duration_s}s ...")

    status = mmwcas.mmw_arming_tda(capture_dir)
    if status != 0:
        print(f"mmw_arming_tda failed (status: {status})")
        return status, capture_dir
    time.sleep(2)

    # IR recording is only ever "on" for the TDA framing window below - armed
    # right before mmw_start_frame(), disarmed the instant the duration sleep
    # ends (before mmw_stop_frame()/de-arm/transfer, which are unrelated to
    # framing and shouldn't pick up stray edges).
    global ir_recording
    if ir_enabled:
        ir_recording = True

    status = mmwcas.mmw_start_frame()
    if status != 0:
        ir_recording = False
        print(f"mmw_start_frame failed (status: {status})")
        return status, capture_dir

    print(f"\n Capturing... ({duration_s}s)")
    time.sleep(duration_s)
    ir_recording = False
    ir_npy_path = save_ir_timestamps(capture_dir)

    status = mmwcas.mmw_stop_frame()
    if status != 0:
        print(f"mmw_stop_frame failed (status: {status})")
        return status, capture_dir

    status = mmwcas.mmw_dearming_tda()
    if status != 0:
        print(f"mmw_dearming_tda failed (status: {status})")
        return status, capture_dir

    # Check if files were actually captured
    print("\n" + "="*60)
    print("Verifying data capture...")
    print("="*60)

    success, file_count, files = check_captured_files(capture_dir, tda_ip)

    if not success:
        print("\n  WARNING: No files found in capture directory!")
        print("\n  Skipping .mmwave.json generation.")
        return status, capture_dir

    # Generate configuration JSON file only if capture was successful
    json_filename = os.path.join("mmwave_json_files", f"{capture_dir}.mmwave.json")
    print(f"\nGenerating configuration file: {json_filename}")
    export_config_to_json(config_dict, json_filename)

    # Push the IR timestamps + config sidecar up into the same TDA capture
    # directory as the raw .bin data, so they travel together through the
    # existing SCP-transfer-then-delete pipeline (pipeline.py) instead of
    # needing separate correlation after the fact.
    upload_files_to_tda([ir_npy_path, json_filename], capture_dir, tda_ip)

    return status, capture_dir


def run_interactive(args):
    """
    REPL loop mirroring mimo.c's --interactive mode: configure once (already
    done by the caller), then repeatedly prompt for an experiment name and
    record - no reconfiguration between captures.
    """
    print("\n=== Interactive multi-capture mode ===")
    print("Radar is configured and the connection to the TDA is live.")
    print("At the prompt, type an experiment name to arm + record, e.g.:")
    print(f"  bridge_test          (uses --duration value: {args.duration:.2f}s)")
    print("  bridge_test 30       (30 seconds instead)")
    print("Type 'quit'/'exit' or leave blank to stop.\n")

    while True:
        if shutdown_flag:
            print("\n Shutdown requested. Exiting interactive mode...")
            break
        try:
            line = input("experiment> ").strip()
        except EOFError:
            break
        if not line or line in ("quit", "exit"):
            break

        parts = line.split()
        exp_name = parts[0]
        try:
            duration_s = float(parts[1]) if len(parts) >= 2 else args.duration
        except ValueError:
            duration_s = args.duration

        status, capture_dir = run_one_capture(exp_name, duration_s, args.tda_ip)
        if status == 0:
            print(f">>> Capture '{capture_dir}' completed successfully.\n")
        else:
            print(f">>> Capture '{capture_dir}' finished with errors (status {status}). Continuing...\n")

    print("Exiting interactive mode.")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='TIDEP-01012 MIMO Cascade Radar Control IMRSL')
    parser.add_argument('-d', '--directory',
                        type=str,
                        default='mmwave_python',
                        help='Base directory name for data capture (default: mmwave_python)')
    parser.add_argument('-t', '--duration',
                        type=float,
                        default=10.0,
                        help='Recording duration in seconds (default: 10.0)')
    parser.add_argument('--tda-ip',
                        type=str,
                        default='192.168.33.180',
                        help='TDA board IP address (default: 192.168.33.180)')
    parser.add_argument('-n', '--num-loops',
                        type=int,
                        default=1,
                        help='Number of capture loops (default: 1, 0 = infinite until Ctrl+C)')
    parser.add_argument('-i', '--inter-loop-time',
                        type=float,
                        default=60.0,
                        help='Delay between capture loops in seconds (default: 60.0)')
    parser.add_argument('-I', '--interactive',
                        action='store_true',
                        help="Configure once, then repeatedly prompt for an experiment name and "
                             "record - no reconfiguration between captures. At the prompt, type "
                             "'<name>' or '<name> <seconds>' to capture, or 'quit'/blank to exit.")
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
                             f"(default: '{DEFAULT_RADAR_CONFIG}'). "
                             'Add new presets as .toml files there, not here.')

    args = parser.parse_args()

    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Validate arguments
    if args.num_loops < 0:
        print("Error: --num-loops must be >= 0")
        sys.exit(1)

    print(f"Capture duration: {args.duration} seconds")
    if args.interactive:
        print("Mode: interactive (configure once, loop on typed experiment names)")
    else:
        print(f"Number of loops  : {'Infinite (until Ctrl+C)' if args.num_loops == 0 else args.num_loops}")
        if args.num_loops != 1:
            print(f"Inter-loop delay : {args.inter_loop_time} seconds")

    # Select the RF/geometry preset (see radar_config.py) for this run.
    global config_dict
    config_dict = get_radar_config(args.radar_config)
    print(f"Radar config     : {args.radar_config}")

    # Configure radar
    status = mmwcas.mmw_set_config(config_dict)
    if status != 0:
        print(f"Configuration error: {status}")
        raise ValueError(f"mmw_set_config failed with status {status}")

    # Initialize radar
    status = mmwcas.mmw_init()
    assert status == 0, ValueError("mmw_init failed")
    time.sleep(2)

    os.makedirs("mmwave_json_files", exist_ok=True)

    if not args.no_ir:
        setup_ir_sensor(args.ir_pin, args.ir_bounce_ms)
    else:
        print("[IR] IR sensor timestamp logging disabled (--no-ir).")

    if args.interactive:
        try:
            run_interactive(args)
        except KeyboardInterrupt:
            print("\n\nInteractive mode interrupted by user")
            sys.exit(130)
        finally:
            teardown_ir_sensor()
        return

    # Automatic capture loop
    loop_count = 0
    infinite_mode = (args.num_loops == 0)

    try:
        while True:
            # Check if we should continue
            if not infinite_mode and loop_count >= args.num_loops:
                break

            if shutdown_flag:
                print("\n Shutdown requested. Exiting capture loop...")
                break

            loop_count += 1

            print("\n" + "="*60)
            print(f"CAPTURE LOOP {loop_count}" + (" (INFINITE MODE)" if infinite_mode else f" of {args.num_loops}"))
            print("="*60)
            print(f"Recording duration: {args.duration} seconds")
            print("="*60)

            status, capture_dir = run_one_capture(args.directory, args.duration, args.tda_ip)
            if status != 0:
                time.sleep(1)
                continue  # Skip to next loop

            print("\n" + "="*60)
            print(f"Data capture {capture_dir} completed successfully!")
            print("="*60)

    except KeyboardInterrupt:
        print("\n\nCapture interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nError during capture: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        teardown_ir_sensor()

if __name__ == "__main__":
    main()
