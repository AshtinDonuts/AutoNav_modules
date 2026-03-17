# Requirements
- Ubuntu 22.04 (Jammy)
- Ros Humble
- Zed SDK (Cuda 12.8)

# Preperation
## Cloning
```bash
git clone https://github.com/AshtinDonuts/AutoNav_modules.git --recursive # Clone this repo and its submodules
cd AutoNav_modules # Enter ws
```
## Install dependecies
```bash
sudo apt update 
source /opt/ros/humble/setup.bash # Source ros
sudo apt install ros-humble-rtabmap-ros # Install rtabmap_ros binaries
rosdep update 
rosdep install --from-paths src --ignore-src -r -y # install dependencies
```
## Build and Run
```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash # source
ros2 launch autonomous_camera camera.launch.py camera_model:=zed2i
```

# Known Issues
<details>
  <summary><b>Error building, message_filters with .h/.hpp errors</b></summary>
  <p>In your apt message_filters file (findable with the command <code>find /opt/ros/humble/include/message_filters</code>), ensure you have both .h and .hpp files of the error type.</p>
</details>

<details>
  <summary><b>I'm getting spammed "rtabmap: Did not receive data since 5 seconds"</b></summary>
  <p> Could be an issue with the broken system ffmpeg transport. Uninstall it using <code>sudo apt remove ros-humble-ffmpeg-image-transport</code>. Then inside the workspace, <code>rm -rf build/zed_components install/zed_components</code> and rebuild zed_components so it stops linking to it </p>
</details>

<!--
<details>
  <summary><b>Question goes here?</b></summary>
  <p>Answer goes here.</p>
</details>
-->


