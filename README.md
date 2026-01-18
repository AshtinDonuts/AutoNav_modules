# Rover ROS2 Workspace

This is a ROS2 workspace for the rover project.

## Workspace Structure

```
rover_ROS_ws/
├── src/          # ROS2 packages go here
├── build/        # Build files (created when building)
├── install/      # Installation files (created when building)
└── log/          # Build logs (created when building)
```

## Setup

### Using Conda (Recommended for Python packages)

If you're using conda environments (e.g., for ZED SDK, PyTorch, etc.):

1. **Activate conda and source ROS2:**
   ```bash
   source setup_conda_ros.sh
   ```
   Or manually:
   ```bash
   # Activate conda environment
   conda activate zed_pytorch  # or your conda env name
   
   # Source ROS2
   source /opt/ros/<ros2-distro>/setup.bash
   
   # Source workspace
   source install/setup.bash
   ```

2. **Fix Python paths in entry point scripts (if needed):**
   ```bash
   ./fix_python_paths.sh
   ```

3. **Build the workspace (with conda activated):**
   ```bash
   cd /home/khw/Dev/rover_ROS_ws
   colcon build
   ```

### Standard Setup (without Conda)

1. Source your ROS2 installation:
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```

2. Build the workspace:
   ```bash
   cd /home/khw/Dev/rover_ROS_ws
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Adding Packages

Place your ROS2 packages in the `src/` directory. Each package should have its own directory with a `package.xml` and `CMakeLists.txt` (for C++ packages) or `setup.py` and `setup.cfg` (for Python packages).

## Building

- Build all packages: `colcon build`
- Build specific package: `colcon build --packages-select <package-name>`
- Build with symlink install: `colcon build --symlink-install`
