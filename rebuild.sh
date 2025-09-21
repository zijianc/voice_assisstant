#!/bin/bash


# # Activate virtual environment
# source venv/bin/activate

# Clean previous build files
rm -rf build install log

# Rebuild workspace
colcon build --packages-select my_voice_assistant

# Source setup
source install/setup.bash

echo "✅ Rebuild complete and environment sourced."
