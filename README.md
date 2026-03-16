# Requirements
- Ubuntu 22.04 (Jammy)
- Ros Humble
- Zed SDK (Cuda 12.8)

# Preperation
## Install dependecies
```bash
sudo apt update 
source /opt/ros/humble/setup.bash # Source ros
```
## Cloning
```bash
git clone https://github.com/AshtinDonuts/AutoNav_modules.git --recursive # Clone this repo and its submodules
cd AutoNav_modules # Enter ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y # install dependencies
```

## Build and Run
```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash # source
ros2 launch mapping_module new_rtab.launch.py camera_model:=zed2i
```

# Output topics
- /zed/zed_node/depth/camera_info
- /zed/zed_node/depth/depth_registered
- /zed/zed_node/depth/depth_registered/camera_info
- /zed/zed_node/depth/depth_registered/compressed
- /zed/zed_node/depth/depth_registered/compressedDepth
- /zed/zed_node/depth/depth_registered/theora
- /zed/zed_node/imu/data
- /zed/zed_node/odom
- /zed/zed_node/point_cloud/cloud_registered
- /zed/zed_node/pose
- /zed/zed_node/pose/status
- /zed/zed_node/rgb/color/rect/camera_info
- /zed/zed_node/rgb/color/rect/image
- /zed/zed_node/rgb/color/rect/image/camera_info
- /zed/zed_node/rgb/color/rect/image/compressed
- /zed/zed_node/rgb/color/rect/image/compressedDepth
- /zed/zed_node/rgb/color/rect/image/theora
- /zed/zed_node/status/health
- /zed/zed_node/status/heartbeat