# Requirements
- Ubuntu 22.04 (Jammy)
- Ros Humble
- Zed SDK (Cuda 12.8)

# Preperation
## Install dependecies
```bash
sudo apt update 
source /opt/ros/humble/setup.bash # Source ros
sudo apt install ros-humble-rtabmap-ros # Install rtabmap_ros binaries
rosdep update 
```
## Cloning
```bash
git clone https://github.com/AshtinDonuts/AutoNav_modules.git --recursive # Clone this repo and its submodules
cd AutoNav_modules # Enter ws
rosdep install --from-paths src --ignore-src -r -y # install dependencies
```

## Build and Run
```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash # source
ros2 launch autonomous_camera camera.launch.py camera_model:=zed2i
```