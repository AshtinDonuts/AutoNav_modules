#!/bin/bash
# Fix Python paths in ROS2 entry point scripts to use conda Python

CONDA_ENV="${CONDA_ENV:-zed_pytorch}"
INSTALL_DIR="$HOME/Dev/rover_ROS_ws/install"

# Find conda Python
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
fi

conda activate "$CONDA_ENV"
CONDA_PYTHON=$(which python3)

if [ -z "$CONDA_PYTHON" ]; then
    echo "Error: Could not find conda Python"
    exit 1
fi

echo "Using conda Python: $CONDA_PYTHON"

# Find all entry point scripts and update their shebang
find "$INSTALL_DIR" -type f \( -name "zed_vslam" -o -name "zed_3d_detector" -o -name "zed_3dpc" -o -name "zed_2d_detector" \) | while read script; do
    if [ -f "$script" ]; then
        echo "Fixing: $script"
        # Replace hardcoded python3 path with env python3 (which will use PATH)
        sed -i "1s|^#!.*python3|#!/usr/bin/env python3|" "$script"
        # Or use conda Python directly
        # sed -i "1s|^#!.*|#!$CONDA_PYTHON|" "$script"
    fi
done

echo "Done! Entry point scripts updated."
