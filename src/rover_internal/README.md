colcon build --packages-select mapping_module
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# Quick start

Launch our 3d pointcloud ROS publisher:

`ros2 launch mapping_module zed_3dpc.launch.py`

Launch our vSLAM ROS publisher:
