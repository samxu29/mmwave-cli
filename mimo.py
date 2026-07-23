"""
mimo.py - TIDEP-01012 MIMO Cascade Radar Control (Python / mmwcas Cython wrapper)

RF/geometry configuration (chirp profiles, idle times, antenna geometry,
frame timing) has moved OUT of this file and into radar_config.py
(2026-07-22) - see that module's header for details and for how to add a new
idle-time scheme or antenna geometry preset without editing this file. Select
a non-default preset with --radar-config <name>.

Capture length is defined by FRAME COUNT (--frames), not wall-clock seconds.
The radar + TDA are programmed with that numFrames so they stop after exactly
N frames (e.g. 300 frames @ 100 ms period ≈ 30 s). Host sleep is only
N × framePeriodicity + a small margin so we do not stop_frame early.

Single capture per process invocation, then exit (used by pipeline.py). For
repeats, re-invoke this script (fresh mmw_init() each time) from pipeline.py,
a shell loop, or cron - do not add a capture loop back into this file (see
the "Single capture, then exit" comment in main() for why).

The old --interactive REPL mode (configure once, loop on typed experiment
names) has moved to mimo_interactive.py and is DEPRECATED - see that file's
docstring.

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
import copy
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


def _frame_period_s(cfg):
    """Frame period in seconds from a radar config dict (periodicity is ms)."""
    return float(cfg["mimo"]["frame"]["framePeriodicity"]) / 1000.0


def _wait_s_for_frames(num_frames, period_s):
    """Host wait after start_frame: N periods + one period margin for TDA flush."""
    return num_frames * period_s + period_s


def _rx_popcount_per_device(cfg):
    """Return [nRx0, nRx1, nRx2, nRx3] from mimo.channel.rxChannelEn
    (scalar broadcasts to all four devices; list is per-device)."""
    rx = cfg["mimo"]["channel"]["rxChannelEn"]
    if isinstance(rx, (list, tuple)):
        return [bin(int(x)).count("1") for x in rx]
    n = bin(int(rx)).count("1")
    return [n, n, n, n]


def _expected_bytes_per_frame_for_device(cfg, dev_id):
    """
    On-disk bytes for ONE frame in a single device's <dev>_0000_data.bin, for
    16-bit complex ADC (the only layout this tool writes - TDA dataPacking is
    hard-coded 0 = 16-bit in mmw_arming_tda()).

        bytes/frame/device =
            numAdcSamples × chirpsPerLoop × numLoops × numRxOnThisDevice
            × 2 (I+Q) × 2 (bytes per 16-bit sample)

    chirpsPerLoop = chirpEndIdx - chirpStartIdx + 1. numRxOnThisDevice is the
    popcount of that device's rxChannelEn (supports per-device lists). Returns
    0 when that device has RX disabled; None if fields are missing/unparseable
    (frame-drop check is then silently skipped).
    """
    try:
        prof = cfg["mimo"]["profile"]
        frame = cfg["mimo"]["frame"]
        num_adc = int(prof["numAdcSamples"])
        num_loops = int(frame["numLoops"])
        chirps_per_loop = int(frame["chirpEndIdx"]) - int(frame["chirpStartIdx"]) + 1
        num_rx = _rx_popcount_per_device(cfg)[dev_id]
        if num_adc <= 0 or num_loops <= 0 or chirps_per_loop <= 0:
            return None
        if num_rx <= 0:
            return 0
        return num_adc * chirps_per_loop * num_loops * num_rx * 2 * 2
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def _frame_active_time_us(cfg):
    """
    Total time the radar spends chirping in ONE frame, in microseconds:

        active = numLoops × Σ_chirps (idleTime[profile] + rampEndTime)

    This must fit inside framePeriodicity (ms) or the device can't honour the
    frame trigger and SKIPS frames (RF frame overrun) - a data-rate-independent
    drop cause. Returns None if fields are missing/unparseable.
    """
    try:
        prof = cfg["mimo"]["profile"]
        frame = cfg["mimo"]["frame"]
        chirp = cfg["mimo"]["chirp"]
        idle_times = prof["idleTimes"]
        ramp = float(prof["rampEndTime"])
        num_loops = int(frame["numLoops"])
        pids = chirp["profileIdPerChirp"]
        per_loop = sum(float(idle_times[int(pid)]) + ramp for pid in pids)
        return per_loop * num_loops
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def _dev_id_from_data_bin(name):
    """Parse leading device index from e.g. 'master_0000_data.bin' /
    'slave1_0000_data.bin' / '0_0000_data.bin'. Returns None if unknown."""
    base = str(name).split("/")[-1].lower()
    if base.startswith("master"):
        return 0
    if base.startswith("slave1") or base.startswith("slave_1"):
        return 1
    if base.startswith("slave2") or base.startswith("slave_2"):
        return 2
    if base.startswith("slave3") or base.startswith("slave_3"):
        return 3
    # Numeric prefix used by some TDA layouts: '0_0000_data.bin'
    head = base.split("_", 1)[0]
    if head.isdigit():
        return int(head)
    return None


def _warn_on_frame_drops(files, num_frames, cfg):
    """
    Cross-check on-disk <dev>_0000_data.bin sizes against the requested frame
    count and print a prominent WARNING if the TDA wrote FEWER frames than
    requested (dropped frames, not wall-clock jitter which is bounded to +/-1
    frame). When frames are missing, also report the chirp-schedule duty cycle
    and diagnose the likely cause: on this hardware the dominant cause is RF
    FRAME OVERRUN (chirp schedule nearly fills framePeriodicity), not TDA->SSD
    bandwidth - proven by the fact that halving the data rate (fewer RX) left
    the drop rate unchanged, while the only low-duty-cycle preset
    (cascade_mimo) stopped dropping. Each device file is checked independently
    (per-device RX masks supported). Best-effort: silently returns if the
    per-frame size can't be derived from cfg or no data files are present.

    Returns the number of dropped frames (max across device files, 0 if none).
    """
    data_files = [(name, int(size)) for name, size in files
                  if name.endswith("_data.bin") and str(size).isdigit()]
    if not data_files:
        return 0

    max_dropped = 0
    lines = []
    any_sized = False
    for name, size in sorted(data_files):
        dev_id = _dev_id_from_data_bin(name)
        bpf = (_expected_bytes_per_frame_for_device(cfg, dev_id)
               if dev_id is not None else None)
        if bpf is None:
            # Fall back: try master mask if name is unrecognised.
            bpf = _expected_bytes_per_frame_for_device(cfg, 0)
        if bpf is None:
            continue
        if bpf == 0:
            lines.append(f"    {name:28s}  RX disabled (expect ~0 bytes, "
                         f"got {size})")
            continue
        any_sized = True
        whole = size // bpf
        partial = (size % bpf) != 0
        dropped = num_frames - whole
        if dropped > 0:
            max_dropped = max(max_dropped, dropped)
        note = ""
        if dropped > 0:
            note = f"   <-- {dropped} dropped ({dropped / num_frames * 100:.1f}%)"
        elif partial:
            note = "   <-- trailing partial frame"
        lines.append(f"    {name:28s} {whole:>5d} frame(s)"
                     + ("+partial" if partial else "        ") + note)

    if not lines:
        return 0

    print(f"\n{'-'*60}")
    print(f"Frame integrity check  (requested {num_frames} frame(s); "
          f"bytes/frame varies by device RX mask)")
    print(f"{'-'*60}")
    for ln in lines:
        print(ln)
    if max_dropped > 0:
        print(f"\n  ** WARNING: {max_dropped} frame(s) short of the requested "
              f"{num_frames}. **")
        # Diagnose cause from the chirp schedule duty cycle. Empirically the
        # dominant drop cause on this hardware is RF frame overrun (chirp
        # schedule nearly fills framePeriodicity), NOT TDA->SSD bandwidth:
        # halving the data rate (fewer RX) left the drop rate unchanged, while
        # only the low-duty-cycle preset (cascade_mimo) stopped dropping.
        active_us = _frame_active_time_us(cfg)
        try:
            period_ms = float(cfg["mimo"]["frame"]["framePeriodicity"])
        except (KeyError, TypeError, ValueError):
            period_ms = None
        if active_us is not None and period_ms:
            active_ms = active_us / 1000.0
            duty = active_ms / period_ms * 100.0
            print(f"     Chirp schedule: {active_ms:.1f} ms active per "
                  f"{period_ms:.0f} ms frame ({duty:.0f}% duty, "
                  f"{period_ms - active_ms:.1f} ms idle).")
            if duty >= 90.0:
                print(f"     -> Almost certainly RF FRAME OVERRUN: the schedule "
                      f"nearly fills (or")
                print(f"        exceeds) framePeriodicity, so the device skips "
                      f"frames. This is NOT")
                print(f"        a bandwidth problem. Fix: raise framePeriodicity, "
                      f"or shorten the")
                print(f"        schedule (smaller idleTimes / rampEndTime / "
                      f"numLoops).")
            else:
                print(f"     -> Schedule has headroom, so this is more likely a "
                      f"TDA->SSD throughput")
                print(f"        hiccup. Reduce the data rate (numLoops / RX / "
                      f"ADC samples / fps).")
        else:
            print(f"     Check the chirp schedule vs framePeriodicity (frame "
                  f"overrun) first, then")
            print(f"     the TDA->SSD data rate.")
    elif any_sized:
        print(f"\n  OK: all RX-enabled device files contain the full "
              f"{num_frames} frame(s).")
    print(f"{'-'*60}")
    return max_dropped

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    _HAS_GPIO = False

# RF/geometry config dict - selected from radar_config.RADAR_CONFIGS in
# main() based on --radar-config (defaults to cascade_tx3_rx16). Set here so
# run_one_capture() (which reads this module-level global) works. Also read/
# reassigned directly by mimo_interactive.py (DEPRECATED REPL wrapper) so
# that script can reuse this module's capture logic unmodified.
config_dict = get_radar_config(DEFAULT_RADAR_CONFIG)

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


def run_one_capture(exp_label, num_frames, tda_ip):
    """
    Arm the TDA, record one capture, de-arm, then verify + export the
    .mmwave.json sidecar. Same division of responsibility as mimo.c's
    run_capture(). Safe to call repeatedly after a single mmw_set_config()/
    mmw_init() - reused by the DEPRECATED mimo_interactive.py REPL for
    exactly that, though repeated same-process captures are the case known
    to show SSD/network throughput drift (see mimo.py's module docstring).

    Capture length is num_frames: the host waits
    num_frames × framePeriodicity + one period margin before calling
    mmw_stop_frame(). This is wall-clock-based, not hardware-exact - actual
    byte counts can vary by a few % between runs of the same num_frames due
    to RPC/network timing jitter.

    NOTE: a live mmw_reconfigure_frame_count() (re-issuing MMWL_frameConfig
    mid-session to hard-stop chips at an exact frame count) was tried here
    and reverted - on real hardware it caused mmw_start_frame() to fail
    (some devices left stuck "frame ongoing") on the very first capture of a
    fresh session, which then wedges ALL subsequent frame-config/start calls
    (RL_RET_CODE_FRAME_IS_ONGOING) until the RF chips are power-cycled /
    process restarted. Not worth the reliability cost - see git history for
    the mmwcas.mmw_reconfigure_frame_count() function kept for reference.

    Returns (status, capture_dir) - status is 0 on full success.
    """
    global config_dict
    timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
    capture_dir = f"{exp_label}_{timestamp}"
    period_s = _frame_period_s(config_dict)
    wait_s = _wait_s_for_frames(num_frames, period_s)
    approx_s = num_frames * period_s

    # So .mmwave.json matches this capture even when interactive overrides
    # the default --frames. Does NOT reprogram the RF chips (see docstring) -
    # actual hardware frame count is whatever was configured at mmw_init()
    # time and does not change per-prompt.
    config_dict["mimo"]["frame"]["numFrames"] = num_frames

    print(f"\n>>> Capturing '{capture_dir}' — {num_frames} frames "
          f"(~{approx_s:.1f}s @ {period_s*1000:.0f} ms/frame) ...")

    status = mmwcas.mmw_arming_tda(capture_dir, num_frames)
    if status != 0:
        print(f"mmw_arming_tda failed (status: {status})")
        return status, capture_dir
    time.sleep(2)

    # IR recording is only ever "on" for the TDA framing window below - armed
    # right before mmw_start_frame(), disarmed when the frame wait ends
    # (before mmw_stop_frame()/de-arm/transfer, which are unrelated to
    # framing and shouldn't pick up stray edges).
    global ir_recording
    if ir_enabled:
        ir_recording = True

    status = mmwcas.mmw_start_frame()
    if status != 0:
        ir_recording = False
        print(f"mmw_start_frame failed (status: {status})")
        return status, capture_dir

    print(f"\n Capturing... ({num_frames} frames, waiting {wait_s:.2f}s)")
    time.sleep(wait_s)
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

    # Compare on-disk frame count vs. requested and warn on dropped frames
    # (TDA/SSD couldn't keep up). Does not fail the capture - the data is
    # still usable, just shorter than asked for.
    _warn_on_frame_drops(files, num_frames, config_dict)

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


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='TIDEP-01012 MIMO Cascade Radar Control IMRSL')
    parser.add_argument('-d', '--directory',
                        type=str,
                        default='mmwave_python',
                        help='Base directory name for data capture (default: mmwave_python)')
    parser.add_argument('--frames',
                        type=int,
                        default=100,
                        help='Number of radar frames to capture (programs frame.numFrames + '
                             'TDA numberOfFramesToCapture). Default: 100. At 100 ms period, '
                             '100 frames ≈ 10 s, 300 frames ≈ 30 s.')
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
                             f"(default: '{DEFAULT_RADAR_CONFIG}'). "
                             'Add new presets as .toml files there, not here.')

    args = parser.parse_args()

    # Register signal handler for Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    # Validate arguments
    if args.frames < 1:
        print("Error: --frames must be >= 1")
        sys.exit(1)

    # Select the RF/geometry preset (see radar_config.py) for this run.
    # deepcopy so CLI --frames can override TOML numFrames without mutating cache.
    global config_dict
    config_dict = copy.deepcopy(get_radar_config(args.radar_config))
    period_ms = float(config_dict["mimo"]["frame"]["framePeriodicity"])
    period_s = period_ms / 1000.0
    config_dict["mimo"]["frame"]["numFrames"] = args.frames
    approx_s = args.frames * period_s

    print(f"Capture frames   : {args.frames}  (~{approx_s:.1f}s @ {period_ms:.0f} ms/frame)")
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

    # Single capture, then exit. Repeated back-to-back captures within the
    # same process (formerly --num-loops/--inter-loop-time) were removed:
    # even with a real cooldown sleep between them, capture sizes for
    # identical --frames still drifted smaller run to run - almost certainly
    # an SSD/network throughput ceiling on the TDA (frame drops), not
    # something fixable from the host side. Run mimo.py again (fresh process,
    # fresh mmw_init()) for each capture, or drive repeats externally (a
    # shell loop, cron, or pipeline.py) so you control the spacing.
    print("\n" + "="*60)
    print(f"Recording frames: {args.frames}  (~{args.frames * period_s:.1f}s)")
    print("="*60)

    try:
        status, capture_dir = run_one_capture(args.directory, args.frames, args.tda_ip)
        if status != 0:
            sys.exit(1)

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
