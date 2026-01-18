#!/bin/bash
# Setup script to activate conda environment before sourcing ROS2

# Default conda environment (can be overridden)
CONDA_ENV="${CONDA_ENV:-zed_pytorch}"

# Initialize conda
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
fi

# Activate conda environment
if [ -n "$CONDA_ENV" ]; then
    conda activate "$CONDA_ENV"
    echo "Activated conda environment: $CONDA_ENV"
    echo "Python: $(which python3)"
    echo "Python version: $(python3 --version)"
fi

# Source ROS2 (adjust path as needed)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
fi

# Source workspace
if [ -f "$HOME/Dev/rover_ROS_ws/install/setup.bash" ]; then
    source "$HOME/Dev/rover_ROS_ws/install/setup.bash"
fi

# Verify Python is from conda
if [[ "$(which python3)" == *"conda"* ]]; then
    echo "✓ Using conda Python: $(which python3)"
else
    echo "⚠ Warning: Python may not be from conda: $(which python3)"
fi
