#!/bin/bash

# Load the .env file
set -a
source .env
set +a

DEVICE=$1

if [[ -z "$DEVICE" ]]; then
  echo "Usage: $0 <DEVICE>"
  echo "Example: $0 LEFT_ARM"
  exit 1
fi

# Construct variable names
PORT_VAR="${DEVICE}_PORT"
# BOARD_VAR="${DEVICE}_BOARD"
PATH_VAR="${DEVICE}_PATH"

# Get the values
PORT="${!PORT_VAR}"
# BOARD="${!BOARD_VAR}"
BOARD=$(grep -oP '(?<=^FQBN\s*=\s*).*' ./src/devices.env | tr -d '"')
PATH="${!PATH_VAR}"

if [[ -z "$PORT" || -z "$BOARD" || -z "$PATH" ]]; then
  echo "Missing environment variables for $DEVICE"
  exit 2
fi

echo "Uploading to $DEVICE"
echo "PORT:  $PORT"
echo "BOARD: $BOARD"
echo "PATH:  $PATH"

# Run compile + upload
arduino-cli compile --fqbn "$BOARD" "$PATH" || exit 3
arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$PATH" || exit 4

echo "✅ Upload complete for $DEVICE"
