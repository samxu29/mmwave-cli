#!/usr/bin/env bash
#
# run_experiment.sh - Run one mmwave capture with synchronized IR sensor
# timestamp logging.
#
# Launches ir_logger.py in the background (GPIO armed immediately), then
# runs `./mmwave -d <exp_name> --configure --record --time <minutes>`,
# watching its live output for the line:
#     STATUS    0 | DEV MAP: 32 | [MMWCAS-DSP] Arming TDA
# At that moment it signals ir_logger.py (SIGUSR1) to start actually
# recording IR rising-edge timestamps. When mmwave exits (capture done,
# frames stopped, TDA de-armed - all raw .bin data stays on the TDA's
# /mnt/ssd as usual, this script does not touch that), ir_logger.py is
# signaled (SIGTERM) to stop and save.
#
# Output (per run):
#   output/<exp_name>/ir_timestamps.txt   one epoch timestamp per line
#   output/<exp_name>/mmwave.log          full mmwave stdout/stderr
#
# Usage:
#   ./run_experiment.sh <exp_name> [duration_minutes]
#
#   duration_minutes defaults to 0.5 (30s), same units as mmwave's --time.
#
# Examples:
#   ./run_experiment.sh bridge_test_01
#   ./run_experiment.sh bridge_test_02 1.0
#
# Config overrides (env vars):
#   IR_PIN=4          BCM GPIO pin of the IR sensor (default: 4, matches
#                     TCRT5000_PIN in receiver_ir.py)
#   IR_BOUNCE_MS=200  Software debounce window in ms (default: 200)
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MMWAVE_BIN="${SCRIPT_DIR}/mmwave"
IR_LOGGER="${SCRIPT_DIR}/ir_logger.py"

IR_PIN="${IR_PIN:-4}"
IR_BOUNCE_MS="${IR_BOUNCE_MS:-200}"

EXP_NAME="${1:?Usage: $0 <exp_name> [duration_minutes]}"
DURATION="${2:-0.5}"

OUT_DIR="${SCRIPT_DIR}/output/${EXP_NAME}"
mkdir -p "$OUT_DIR"
IR_OUT="${OUT_DIR}/ir_timestamps.txt"
MMWAVE_LOG="${OUT_DIR}/mmwave.log"

if [[ ! -x "$MMWAVE_BIN" ]]; then
  echo "Error: mmwave binary not found or not executable at $MMWAVE_BIN" >&2
  echo "Build it first with: make all" >&2
  exit 1
fi

echo "[run_experiment] Starting IR logger (pin ${IR_PIN}, bounce ${IR_BOUNCE_MS}ms)..."
python3 "$IR_LOGGER" "$IR_OUT" --pin "$IR_PIN" --bounce-ms "$IR_BOUNCE_MS" &
IR_PID=$!

cleanup() {
  if kill -0 "$IR_PID" 2>/dev/null; then
    kill -TERM "$IR_PID" 2>/dev/null
    wait "$IR_PID" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

# Give ir_logger.py a moment to finish GPIO setup before arming the radar.
sleep 1
if ! kill -0 "$IR_PID" 2>/dev/null; then
  echo "Error: ir_logger.py exited immediately - check GPIO permissions/wiring." >&2
  exit 1
fi

echo "[run_experiment] Capturing '${EXP_NAME}' for ${DURATION} min..."
echo "[run_experiment] Waiting to see TDA arming before starting IR recording."

STARTED=0
# stdbuf -oL forces line-buffered stdout so the watcher below sees each
# STATUS line as it's printed, instead of only after mmwave exits
# (mmwave's own printf calls are otherwise fully-buffered once stdout is
# a pipe rather than a terminal).
stdbuf -oL -eL "$MMWAVE_BIN" -d "$EXP_NAME" --configure --record --time "$DURATION" 2>&1 \
  | tee "$MMWAVE_LOG" \
  | while IFS= read -r line; do
      echo "$line"
      if [[ "$STARTED" -eq 0 ]] && [[ "$line" == *"Arming TDA"* ]]; then
        echo "[run_experiment] TDA arming detected - starting IR timestamp recording."
        kill -USR1 "$IR_PID" 2>/dev/null
        STARTED=1
      fi
    done
MMWAVE_STATUS="${PIPESTATUS[0]}"

echo "[run_experiment] mmwave finished (exit ${MMWAVE_STATUS}). Stopping IR logger..."
cleanup
trap - EXIT INT TERM

echo "[run_experiment] Done."
echo "[run_experiment]   IR timestamps : $IR_OUT"
echo "[run_experiment]   mmwave log    : $MMWAVE_LOG"

exit "$MMWAVE_STATUS"
