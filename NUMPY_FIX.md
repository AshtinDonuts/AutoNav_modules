# NumPy Version Incompatibility Fix

## Problem

The ZED SDK was compiled with NumPy 1.x, but your system has NumPy 2.2.6. This causes:
- `AttributeError: _ARRAY_API not found`
- `array assignment index out of range`
- Node crashes when trying to process frames

## Solution

Downgrade NumPy to version < 2.0:

```bash
pip install 'numpy<2.0'
```

Or if using conda:

```bash
conda install 'numpy<2.0'
```

## Verify

After installing, verify the version:

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

You should see a version like `1.26.x` or `1.24.x`, not `2.x`.

## Alternative: Use Conda Environment

If you have a conda environment with NumPy 1.x already set up, you can use that:

```bash
conda activate your_env_name
# Then launch the node
ros2 launch mapping_module zed_vslam_rtabmap.launch.py
```

## After Fixing

Once NumPy is downgraded, rebuild and relaunch:

```bash
cd ~/Dev/rover_ROS_ws
colcon build --packages-select mapping_module
source install/setup.bash
ros2 launch mapping_module zed_vslam_rtabmap.launch.py
```

The node should now work correctly and publish RGB/depth images to rtabmap.
