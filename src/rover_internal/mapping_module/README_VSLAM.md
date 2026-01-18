# ZED VSLAM Module with Live Occupancy Tracking

This module provides a Visual SLAM (VSLAM) system using the ZED 2 SDK with live occupancy grid mapping. It combines ZED's positional tracking with area memory for loop closure and relocalization, and builds a real-time 2D occupancy grid map from depth data.

## Features

- **Positional Tracking**: Uses ZED SDK's positional tracking with area memory for loop closure
- **Occupancy Grid Mapping**: Real-time 2D occupancy grid generation from point cloud data
- **Pose Publishing**: Publishes camera pose, odometry, and TF transforms
- **SVO Playback**: Support for replaying recorded SVO files
- **Area Memory**: Load and save persistent area maps for relocalization

## Prerequisites

### 1. ZED SDK Installation

Install the ZED SDK and Python API following the official instructions:
- [ZED SDK Installation](https://www.stereolabs.com/docs/installation/)
- [Python API Installation](https://www.stereolabs.com/docs/installation/python/)

```bash
# Install pyzed Python package
pip3 install pyzed
```

### 2. ROS2 Dependencies

The following ROS2 packages are required:
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `cv_bridge`
- `launch`
- `launch_ros`

These should be installed with your ROS2 distribution.

### 3. Python Dependencies

```bash
pip3 install numpy opencv-python
```

## Building the Package

1. Navigate to your workspace:
```bash
cd /home/khw/Dev/rover_ROS_ws
```

2. Build the package:
```bash
colcon build --packages-select mapping_module
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running with Live Camera

Launch the VSLAM node with default parameters:

```bash
ros2 run mapping_module zed_vslam
```

Or with custom parameters:

```bash
ros2 run mapping_module zed_vslam \
    --grid-resolution 0.05 \
    --grid-width 200 \
    --grid-height 200 \
    --min-height -0.5 \
    --max-height 2.0 \
    --publish-rate 10.0
```

### Running with SVO File

To replay a recorded SVO file:

```bash
ros2 run mapping_module zed_vslam \
    --svo-filename /path/to/recording.svo \
    --area-file /path/to/map.area
```

### Using Launch File

Launch with the provided launch file:

```bash
ros2 launch mapping_module zed_vslam.launch.py
```

With custom parameters:

```bash
ros2 launch mapping_module zed_vslam.launch.py \
    svo_file:=/path/to/recording.svo \
    area_file:=/path/to/map.area \
    grid_resolution:=0.05 \
    grid_width:=200 \
    grid_height:=200
```

### Command Line Arguments

- `--svo-filename`: Path to SVO file to replay (optional, for live camera leave empty)
- `--area-file`: Path to area memory file (.area) for loading/saving persistent maps
- `--grid-resolution`: Occupancy grid resolution in meters (default: 0.05)
- `--grid-width`: Occupancy grid width in cells (default: 100)
- `--grid-height`: Occupancy grid height in cells (default: 100)
- `--min-height`: Minimum height filter in meters (default: -0.5)
- `--max-height`: Maximum height filter in meters (default: 2.0)
- `--publish-rate`: Publishing rate in Hz (default: 10.0)

## ROS2 Topics

### Published Topics

- `/zed_vslam/occupancy_grid` (nav_msgs/OccupancyGrid): 2D occupancy grid map
- `/zed_vslam/pose` (geometry_msgs/PoseStamped): Camera pose in map frame
- `/zed_vslam/odometry` (nav_msgs/Odometry): Camera odometry
- `/zed_vslam/point_cloud` (sensor_msgs/PointCloud2): Point cloud data (optional)
- `/zed_vslam/camera_info` (sensor_msgs/CameraInfo): Camera calibration parameters

### TF Transforms

- `map` → `zed_camera_center`: Transform from map frame to camera frame

## Configuration

### Occupancy Grid Parameters

- **Grid Resolution**: Lower values (0.02-0.05m) provide higher detail but require more memory
- **Grid Size**: Larger grids (200x200+) allow mapping larger areas but use more memory
- **Height Filtering**: Adjust `min_height` and `max_height` based on your environment:
  - For ground robots: `-0.5` to `2.0` meters typically works well
  - For aerial robots: Adjust based on flight altitude

### Positional Tracking

The module uses ZED's positional tracking with:
- **Area Memory**: Enabled by default for loop closure and relocalization
- **Pose Smoothing**: Enabled for smoother pose estimates
- **IMU Fusion**: Enabled for better tracking accuracy

### Area Memory Files

Area memory files (`.area`) store the spatial memory of the environment:
- **Saving**: Area memory is automatically saved on shutdown if `--area-file` is provided
- **Loading**: Provide `--area-file` on startup to load a previously saved map
- **Relocalization**: When loading an area file, the camera can relocalize in previously mapped areas

## Integration with Navigation

The occupancy grid can be used with ROS2 navigation stacks:

1. **Nav2**: The occupancy grid is published in the standard `nav_msgs/OccupancyGrid` format
2. **Frame Convention**: Uses `map` frame for the occupancy grid
3. **TF Tree**: Publishes transforms from `map` to `zed_camera_center`

Example integration with Nav2:

```bash
# Terminal 1: Launch VSLAM
ros2 launch mapping_module zed_vslam.launch.py

# Terminal 2: Launch Nav2 (configured to use /zed_vslam/occupancy_grid)
ros2 launch nav2_bringup navigation2.launch.py
```

## Testing with SVO Files

As per ZED documentation, you can test the VSLAM system with recorded SVO files:

1. **Record an SVO file** using ZED SDK tools or the ZED ROS wrapper
2. **Replay the SVO file**:
   ```bash
   ros2 run mapping_module zed_vslam --svo-filename /path/to/recording.svo
   ```
3. **Generate area map**: After processing, the area memory will be saved if `--area-file` is provided

## Troubleshooting

### ZED SDK Not Found

If you see "ZED SDK not available":
```bash
# Verify ZED SDK installation
python3 -c "import pyzed.sl as sl; print('ZED SDK OK')"

# If not installed, follow ZED SDK installation instructions
```

### Camera Not Opening

- Check that the ZED camera is connected via USB 3.0
- Verify camera permissions: `lsusb` should show the ZED camera
- Check ZED SDK installation and camera drivers

### Poor Tracking Quality

- Ensure good lighting conditions
- Avoid repetitive textures
- Enable area memory for better tracking
- Check that depth mode is set appropriately (NEURAL recommended)

### Occupancy Grid Not Updating

- Check that tracking state is `OK` (check node logs)
- Verify depth data is available
- Adjust height filtering parameters if points are being filtered out

### Memory Issues

- Reduce grid size or resolution if running out of memory
- Process fewer points per frame (adjust step size in code)
- Use lower resolution depth mode

## Performance Tips

1. **Grid Resolution**: Use 0.05-0.10m for most applications
2. **Grid Size**: Start with 100x100 and increase as needed
3. **Publishing Rate**: 5-10 Hz is usually sufficient
4. **Point Sampling**: The code processes every 2nd point by default (adjust `step` in `update_occupancy_grid`)

## References

- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [ZED Positional Tracking](https://www.stereolabs.com/docs/positional-tracking/)
- [ZED Area Memory](https://www.stereolabs.com/docs/positional-tracking/area-memory/)
- [ZED ROS2 Documentation](https://www.stereolabs.com/docs/ros2/)
- [Nav2 Documentation](https://navigation.ros.org/)

## License

TODO: Add license information
