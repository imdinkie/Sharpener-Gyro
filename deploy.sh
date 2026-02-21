#!/usr/bin/env bash
set -euo pipefail

# Deploy this MicroPython project to an ESP32 using mpremote.
# Usage:
#   ./deploy.sh                # auto-detect /dev/ttyUSB* or /dev/ttyACM*
#   ./deploy.sh /dev/ttyUSB0   # explicit port

PORT="${1:-}"

if [[ -z "$PORT" ]]; then
  for p in /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$p" ]]; then
      PORT="$p"
      break
    fi
  done
fi

if [[ -z "$PORT" ]]; then
  echo "No ESP32 serial port found. Connect board and retry."
  echo "Expected devices like /dev/ttyUSB0 or /dev/ttyACM0"
  exit 1
fi

if [[ ! -e "$PORT" ]]; then
  echo "Port does not exist: $PORT"
  exit 1
fi

if [[ ! -x .venv/bin/mpremote ]]; then
  echo "mpremote not found in .venv. Run: source .venv/bin/activate && pip install mpremote"
  exit 1
fi

echo "Using port: $PORT"

. .venv/bin/activate

# Optional quick probe: this fails fast if board is not running MicroPython.
mpremote connect "$PORT" exec "import sys; print(sys.implementation)"

# Copy project files expected by this repo.
mpremote connect "$PORT" cp main.py :main.py
mpremote connect "$PORT" cp reader.py :reader.py
mpremote connect "$PORT" cp index.html :index.html
mpremote connect "$PORT" cp settings.html :settings.html

# Reset so new code starts.
mpremote connect "$PORT" exec "import machine; machine.reset()"

echo "Deployment complete."
