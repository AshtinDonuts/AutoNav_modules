# Installing rtabmap_ros for ROS2 Humble

## Option 1: Install from Binaries (Recommended - Easiest)

This is the simplest method and recommended for most users:

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros
```

After installation, source your ROS environment:
```bash
source /opt/ros/humble/setup.bash
```

## Option 2: Install from Source

The repositories have been cloned to your workspace. To complete the installation:

### 1. Install System Dependencies

First, make sure you have the necessary build tools:

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libsqlite3-dev \
    libvtk9-dev \
    libproj-dev \
    libqt5opengl5-dev \
    qtbase5-dev \
    libqt5svg5-dev
```

### 2. Install ROS Dependencies

```bash
cd ~/Dev/rover_ROS_ws
rosdep update
sudo rosdep install --from-paths src --ignore-src -r -y
```

**Note:** Some optional dependencies may fail to install (like gtsam, libg2o). These are optional and rtabmap_ros will build without them, but with reduced functionality.

### 3. Build the Workspace

Build rtabmap first, then rtabmap_ros:

```bash
cd ~/Dev/rover_ROS_ws

# Build rtabmap core library
colcon build --packages-select rtabmap --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build rtabmap_ros (depends on rtabmap)
colcon build --packages-select rtabmap_ros --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or build everything at once
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Note:** The build process can take 10-30 minutes depending on your system.

### 4. Source the Workspace

```bash
source install/setup.bash
```

## Verify Installation

After installation, verify rtabmap_ros is available:

```bash
ros2 pkg list | grep rtabmap
```

You should see packages like:
- rtabmap_ros
- rtabmap_launch
- rtabmap_slam
- etc.

## Usage

To launch rtabmap_ros with your ZED camera node:

```bash
# Terminal 1: Launch rtabmap_ros
ros2 launch rtabmap_launch rtabmap.launch.py

# Terminal 2: Launch your ZED VSLAM node
ros2 run mapping_module zed_vslam2-node
```

Or create a combined launch file that launches both nodes together.

## Troubleshooting

### CMake Issues

If you get `ModuleNotFoundError: No module named 'cmake'`, you may have a Python cmake wrapper interfering. Use the system cmake:

```bash
# Check which cmake is being used
which cmake

# If it points to ~/.local/bin/cmake, use system cmake directly
/usr/bin/cmake --version

# Or temporarily remove the local cmake from PATH
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
```

### Build Errors

If you encounter build errors:
1. Make sure all dependencies are installed
2. Check that you have enough disk space
3. Try building with fewer parallel jobs: `colcon build --parallel-workers 2`
4. Check that system cmake is installed: `sudo apt install cmake`

### Missing Dependencies

Some optional dependencies may not be available in ROS2 Humble binaries. These are optional:
- `ros-humble-gtsam` - For GTSAM optimization (optional)
- `ros-humble-libg2o` - For g2o optimization (optional)
- `ros-humble-grid-map-ros` - For grid map support (optional)

rtabmap_ros will build and work without these, but with reduced functionality.

For more information, see: https://github.com/introlab/rtabmap_ros
