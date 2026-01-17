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
