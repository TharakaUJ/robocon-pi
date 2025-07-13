#!/bin/bash

# setup.sh - Setup script for robocon-pi

set -euo pipefail

# Your setup commands go here

echo "Setup script started."

# Example: Update package list
# sudo apt-get update
source /opt/ros/humble/setup.bash
source ./install/setup.bash



echo "Setup script completed."