# Requirements
- Ubuntu 22.04 (Jammy)
- Ros Humble

# Preperation
## Install dependecies
```bash
sudo apt update 
source /opt/ros/humble/setup.bash # Source ros
rosdep update

```

## Cloning
```bash
git clone https://github.com/AshtinDonuts/AutoNav_modules.git --recursive # Clone this repo and its submodules
cd AutoNav_modules # Enter ws
rosdep install --from-paths src --ignore-src -r -y # install dependencies
```

## Build
```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
```
