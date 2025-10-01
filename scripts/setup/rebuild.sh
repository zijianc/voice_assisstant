#!/bin/bash


# # Activate virtual environment
source venv/bin/activate

# Clean previous build artifacts
rm -rf build install log

# Build the workspace
colcon build

# Source the environment setup
source install/setup.bash

echo "✅ Rebuild complete and environment sourced."
