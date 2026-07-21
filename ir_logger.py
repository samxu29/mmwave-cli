#!/usr/bin/env python3
"""
ir_logger.py - Standalone IR sensor rising-edge timestamp logger.

Companion to run_experiment.sh: sits in the background while `mmwave`
configures/arms/records on the TDA board, and records one Unix epoch
timestamp per rising edge detected on the IR sensor's GPIO pin (mirrors
the RPi.GPIO callback + time.time() approach used in receiver_ir.py, but
as a small standalone process controlled entirely by signals so a shell
script can start/stop it around an mmwave capture).

Usage:
    python3 ir_logger.py <output_file.txt> [--pin N] [--bounce-ms N]

    <output_file.txt>  Where to write timestamps: one Unix epoch second
                        (%.6f) per line - directly loadable with
                        `numpy.loadtxt()`.
    --pin N             BCM GPIO pin of the IR sensor. Default: 4
                        (matches TCRT5000_PIN in receiver_ir.py).
    --bounce-ms N       Software debounce window in ms. Default: 200
                        (matches receiver_ir.py's bouncetime).

Signals (this is the whole control protocol - see run_experiment.sh):
    SIGUSR1  Start actually recording timestamps. GPIO edge detection is
             armed immediately at process start (before this signal), so
             no edges are missed while waiting for the trigger - they are
             just discarded until SIGUSR1 is received.
    SIGTERM  Stop recording, write all recorded timestamps to
             <output_file.txt>, clean up GPIO, and exit.
"""
import argparse
import signal
import sys
import time

import RPi.GPIO as GPIO

recording = False
timestamps = []
output_path = None


def sensor_callback(channel):
    if recording:
        timestamps.append(time.time())


def start_recording(signum, frame):
    global recording
    recording = True
    print("[ir_logger] SIGUSR1 received - recording started.", flush=True)


def stop_and_save(signum, frame):
    global recording
    recording = False
    with open(output_path, "w") as f:
        for ts in timestamps:
            f.write(f"{ts:.6f}\n")
    print(f"[ir_logger] SIGTERM received - saved {len(timestamps)} "
          f"marker(s) to {output_path}", flush=True)
    GPIO.cleanup()
    sys.exit(0)


def main():
    global output_path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="Path to write timestamps to")
    parser.add_argument("--pin", type=int, default=4,
                         help="BCM GPIO pin of the IR sensor (default: 4)")
    parser.add_argument("--bounce-ms", type=int, default=200,
                         help="Debounce window in ms (default: 200)")
    args = parser.parse_args()
    output_path = args.output

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(args.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(
        args.pin, GPIO.RISING, callback=sensor_callback, bouncetime=args.bounce_ms
    )

    signal.signal(signal.SIGUSR1, start_recording)
    signal.signal(signal.SIGTERM, stop_and_save)

    print(f"[ir_logger] Ready - watching pin {args.pin} for rising edges "
          f"(debounce {args.bounce_ms}ms). Waiting for SIGUSR1 to start...",
          flush=True)

    while True:
        signal.pause()


if __name__ == "__main__":
    main()
