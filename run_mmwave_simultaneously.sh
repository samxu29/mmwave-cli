#!/bin/bash

# --- Variable configuration ---

# Replace 'your_device_name' with the appropriate value for $NAME.
# You can also change this to prompt the user for input instead.
NAME="SimultantRecordTest_$(date +%y%m%d_%H%M%S)"

# Shared config file
CONFIG_FILE="config/Cascade_Configuration_250227_79Ghz_30frame.toml"

# Shared recording duration
TIME_DURATION="0.17"

echo "Starting simultaneous mmwave execution with NAME=$NAME"
echo "--------------------------------------------------------"

# --- Command 1 (run in background) ---
echo "Starting command 1 (IP 192.168.33.180)..."
./mmwave -d "$NAME" -i 192.168.33.180 -f "$CONFIG_FILE" --configure --record --time "$TIME_DURATION" &

# Save PID of the first process
PID1=$!

# --- Command 2 (run in background) ---
echo "Starting command 2 (IP 192.168.33.181)..."
./mmwave -d "$NAME" -i 192.168.33.181 -f "$CONFIG_FILE" --configure --record --time "$TIME_DURATION" &

# Save PID of the second process
PID2=$!

# --- Wait for both commands to finish ---
echo ""
echo "Waiting for both mmwave processes (PID $PID1 and $PID2) to finish..."
wait $PID1
wait $PID2

echo ""
echo "Both mmwave commands have finished executing."
