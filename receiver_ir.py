import sys
import threading
import time

import numpy as np
import RPi.GPIO as GPIO  # Added for TCRT5000

from realtime_capture import adcCapThread

# --- 1. GPIO Setup for TCRT5000 ---
TCRT5000_PIN = 4
gpio_timestamps = []


def sensor_callback(channel):
    """Interrupt callback triggered by the TCRT5000."""
    # Capture the time immediately to minimize latency
    trigger_time = time.time()
    gpio_timestamps.append(trigger_time)


GPIO.setmode(GPIO.BCM)
GPIO.setup(TCRT5000_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# bouncetime=200 ignores subsequent triggers for 200ms to prevent multiple rapid records from one physical event
GPIO.add_event_detect(
    TCRT5000_PIN, GPIO.FALLING, callback=sensor_callback, bouncetime=200
)
# ----------------------------------

a = adcCapThread(1, "adc", receiverType="frame")
a.start()
counter = 0
lostPacketList = []
ItemNumList = []
timestampList = []
frameByteBuffer = bytearray()
t = time.time()
time_10s = 0

output_prefix = sys.argv[1]
timestamp_label = str(int(t))
bin_path = f"{output_prefix}{timestamp_label}.bin"
timestamp_path = f"{output_prefix}{timestamp_label}_timestamps.npy"
gpio_timestamp_path = f"{output_prefix}{timestamp_label}_gpio_markers.npy"

f = open(bin_path, "wb")

# Parse Duration and Start Time
try:
    duration = float(sys.argv[2])  # Allow float for precision (e.g. 30.5)
except IndexError:
    duration = 30.0  # Default fallback

start_time = time.time()
end_time = start_time + duration

print(f"[Radar] Recording for {duration} seconds...")

try:
    while True:
        # Get data from the capture thread
        readItem, ItemNum, lostPacketListFlag, frameTimestamp = a.getFrame()

        if ItemNum > 0:
            lostPacketList.append(lostPacketListFlag)
            ItemNumList.append(ItemNum)
            timestampList.append(frameTimestamp)

            # Write binary data
            f.write(readItem.tobytes())
            counter += 1

        elif ItemNum == -1:
            pass
        elif ItemNum == -2:
            # No data yet, sleep briefly to save CPU
            time.sleep(0.001)

        # CHECK REAL TIME
        if time.time() >= end_time:
            print(f"\n[Radar] Time limit of {duration}s reached.")
            break

except KeyboardInterrupt:
    print("\n[Radar] Interrupted by user.")

finally:
    # Cleanup
    a.whileSign = False
    f.close()

    # Save Radar Frame timestamps
    if timestampList:
        np.save(timestamp_path, np.array(timestampList, dtype=np.float64))

    # Save GPIO Marker timestamps
    if gpio_timestamps:
        np.save(gpio_timestamp_path, np.array(gpio_timestamps, dtype=np.float64))

    # Clean up GPIO pins to prevent warnings on next run
    GPIO.cleanup()

    # --- Run Summary Report ---
    print("\n" + "=" * 40)
    print("           RUN SUMMARY REPORT           ")
    print("=" * 40)
    print(f"**Binary Frames Captured** : {counter}")
    print(
        f"**Radar Timestamps Saved** : {len(timestampList)} (to {timestamp_path if timestampList else 'N/A'})"
    )
    print(
        f"**GPIO Markers Saved** : {len(gpio_timestamps)} (to {gpio_timestamp_path if gpio_timestamps else 'N/A'})"
    )
    print("=" * 40)

print("\a")
