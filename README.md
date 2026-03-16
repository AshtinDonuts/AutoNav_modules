# Requirements
- Ubuntu 22.04 (Jammy)
- Ros Humble
- Zed SDK 5.0.7
- Cuda 12.6.8

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
# NOTE! Building Rtabmap is heavy and will crash on many computers. 
# If your computer is old, try "--parallel-workers 1" instead (slower build time) 
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash # source
ros2 launch mapping_module new_rtab.launch.py camera_model:=zed2i
```

# Output topics
- /zed/joint_states
- /zed/robot_description
- /zed/zed_node/atm_press
- /zed/zed_node/confidence/camera_info
- /zed/zed_node/confidence/confidence_map
- /zed/zed_node/confidence/confidence_map/compressed
- /zed/zed_node/confidence/confidence_map/compressedDepth
- /zed/zed_node/confidence/confidence_map/ffmpeg
- /zed/zed_node/confidence/confidence_map/theora
- /zed/zed_node/depth/camera_info
- /zed/zed_node/depth/depth_info
- /zed/zed_node/depth/depth_registered
- /zed/zed_node/depth/depth_registered/compressed
- /zed/zed_node/depth/depth_registered/compressedDepth
- /zed/zed_node/depth/depth_registered/ffmpeg
- /zed/zed_node/depth/depth_registered/theora
- /zed/zed_node/disparity/disparity_image
- /zed/zed_node/imu/data
- /zed/zed_node/imu/data_raw
- /zed/zed_node/imu/mag
- /zed/zed_node/left/camera_info
- /zed/zed_node/left/image_rect_color
- /zed/zed_node/left/image_rect_color/compressed
- /zed/zed_node/left/image_rect_color/compressedDepth
- /zed/zed_node/left/image_rect_color/ffmpeg
- /zed/zed_node/left/image_rect_color/theora
- /zed/zed_node/left_cam_imu_transform
- /zed/zed_node/left_gray/camera_info
- /zed/zed_node/left_gray/image_rect_gray
- /zed/zed_node/left_gray/image_rect_gray/compressed
- /zed/zed_node/left_gray/image_rect_gray/compressedDepth
- /zed/zed_node/left_gray/image_rect_gray/ffmpeg
- /zed/zed_node/left_gray/image_rect_gray/theora
- /zed/zed_node/left_raw/camera_info
- /zed/zed_node/left_raw/image_raw_color
- /zed/zed_node/left_raw/image_raw_color/compressed
- /zed/zed_node/left_raw/image_raw_color/compressedDepth
- /zed/zed_node/left_raw/image_raw_color/ffmpeg
- /zed/zed_node/left_raw/image_raw_color/theora
- /zed/zed_node/left_raw_gray/camera_info
- /zed/zed_node/left_raw_gray/image_raw_gray
- /zed/zed_node/left_raw_gray/image_raw_gray/compressed
- /zed/zed_node/left_raw_gray/image_raw_gray/compressedDepth
- /zed/zed_node/left_raw_gray/image_raw_gray/ffmpeg
- /zed/zed_node/left_raw_gray/image_raw_gray/theora
- /zed/zed_node/odom
- /zed/zed_node/path_map
- /zed/zed_node/path_odom
- /zed/zed_node/plane
- /zed/zed_node/plane_marker
- /zed/zed_node/point_cloud/cloud_registered
- /zed/zed_node/point_cloud/cloud_registered/draco
- /zed/zed_node/point_cloud/cloud_registered/zlib
- /zed/zed_node/point_cloud/cloud_registered/zstd
- /zed/zed_node/pose
- /zed/zed_node/pose/status
- /zed/zed_node/pose_with_covariance
- /zed/zed_node/rgb/camera_info
- /zed/zed_node/rgb/image_rect_color
- /zed/zed_node/rgb/image_rect_color/compressed
- /zed/zed_node/rgb/image_rect_color/compressedDepth
- /zed/zed_node/rgb/image_rect_color/ffmpeg
- /zed/zed_node/rgb/image_rect_color/theora
- /zed/zed_node/rgb_gray/camera_info
- /zed/zed_node/rgb_gray/image_rect_gray
- /zed/zed_node/rgb_gray/image_rect_gray/compressed
- /zed/zed_node/rgb_gray/image_rect_gray/compressedDepth
- /zed/zed_node/rgb_gray/image_rect_gray/ffmpeg
- /zed/zed_node/rgb_gray/image_rect_gray/theora
- /zed/zed_node/rgb_raw/camera_info
- /zed/zed_node/rgb_raw/image_raw_color
- /zed/zed_node/rgb_raw/image_raw_color/compressed
- /zed/zed_node/rgb_raw/image_raw_color/compressedDepth
- /zed/zed_node/rgb_raw/image_raw_color/ffmpeg
- /zed/zed_node/rgb_raw/image_raw_color/theora
- /zed/zed_node/rgb_raw_gray/camera_info
- /zed/zed_node/rgb_raw_gray/image_raw_gray
- /zed/zed_node/rgb_raw_gray/image_raw_gray/compressed
- /zed/zed_node/rgb_raw_gray/image_raw_gray/compressedDepth
- /zed/zed_node/rgb_raw_gray/image_raw_gray/ffmpeg
- /zed/zed_node/rgb_raw_gray/image_raw_gray/theora
- /zed/zed_node/right/camera_info
- /zed/zed_node/right/image_rect_color
- /zed/zed_node/right/image_rect_color/compressed
- /zed/zed_node/right/image_rect_color/compressedDepth
- /zed/zed_node/right/image_rect_color/ffmpeg
- /zed/zed_node/right/image_rect_color/theora
- /zed/zed_node/right_gray/camera_info
- /zed/zed_node/right_gray/image_rect_gray
- /zed/zed_node/right_gray/image_rect_gray/compressed
- /zed/zed_node/right_gray/image_rect_gray/compressedDepth
- /zed/zed_node/right_gray/image_rect_gray/ffmpeg
- /zed/zed_node/right_gray/image_rect_gray/theora
- /zed/zed_node/right_raw/camera_info
- /zed/zed_node/right_raw/image_raw_color
- /zed/zed_node/right_raw/image_raw_color/compressed
- /zed/zed_node/right_raw/image_raw_color/compressedDepth
- /zed/zed_node/right_raw/image_raw_color/ffmpeg
- /zed/zed_node/right_raw/image_raw_color/theora
- /zed/zed_node/right_raw_gray/camera_info
- /zed/zed_node/right_raw_gray/image_raw_gray
- /zed/zed_node/right_raw_gray/image_raw_gray/compressed
- /zed/zed_node/right_raw_gray/image_raw_gray/compressedDepth
- /zed/zed_node/right_raw_gray/image_raw_gray/ffmpeg
- /zed/zed_node/right_raw_gray/image_raw_gray/theora
- /zed/zed_node/roi_mask/camera_info
- /zed/zed_node/status/health
- /zed/zed_node/status/heartbeat
- /zed/zed_node/stereo/image_rect_color
- /zed/zed_node/stereo/image_rect_color/compressed
- /zed/zed_node/stereo/image_rect_color/compressedDepth
- /zed/zed_node/stereo/image_rect_color/ffmpeg
- /zed/zed_node/stereo/image_rect_color/theora
- /zed/zed_node/stereo_raw/image_raw_color
- /zed/zed_node/stereo_raw/image_raw_color/compressed
- /zed/zed_node/stereo_raw/image_raw_color/compressedDepth
- /zed/zed_node/stereo_raw/image_raw_color/ffmpeg
- /zed/zed_node/stereo_raw/image_raw_color/theora
- /zed/zed_node/temperature/imu
- /zed/zed_node/temperature/left
- /zed/zed_node/temperature/right