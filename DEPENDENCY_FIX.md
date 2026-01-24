# Dependency Conflict Resolution

## Problem

You have a dependency conflict:
- **ZED SDK** requires NumPy < 2.0 (compiled with NumPy 1.x)
- **OpenCV 4.13.0.90** requires NumPy >= 2.0

## Solution

Downgrade OpenCV to a version that works with NumPy 1.x. OpenCV 4.8.x or 4.9.x are compatible with NumPy 1.x.

### Option 1: Downgrade OpenCV (Recommended)

Since you already have NumPy 1.26.4 installed, just downgrade OpenCV:

```bash
pip install --user 'opencv-python<4.10' --upgrade
```

Or if you need system-wide installation (may require sudo):

```bash
pip install 'opencv-python<4.10' --upgrade
```

This will install:
- OpenCV 4.9.x (compatible with NumPy 1.x)
- Keep NumPy 1.26.4 (already installed, compatible with ZED SDK)

### Option 2: Use Specific Versions

```bash
pip install 'opencv-python==4.9.0.80' 'numpy==1.26.4'
```

### Option 3: Use Conda (Best for ZED SDK)

If you have a conda environment, use conda to manage dependencies:

```bash
conda install opencv numpy=1.26
```

## Verify Installation

After installing, verify versions:

```bash
python3 -c "import numpy; import cv2; print(f'NumPy: {numpy.__version__}'); print(f'OpenCV: {cv2.__version__}')"
```

You should see:
- NumPy: 1.26.x (or similar 1.x version)
- OpenCV: 4.9.x (or similar < 4.10 version)

## After Fixing

Rebuild and relaunch:

```bash
cd ~/Dev/rover_ROS_ws
colcon build --packages-select mapping_module
source install/setup.bash
ros2 launch mapping_module zed_vslam_rtabmap.launch.py
```

## Note

If you're using ROS2 Humble's system packages, you might also need to check if `cv_bridge` has any NumPy requirements. The ROS2 packages should be compatible, but if you installed opencv-python via pip, make sure it's compatible with the system NumPy version.
