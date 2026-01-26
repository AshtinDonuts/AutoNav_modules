# How to set up

First build rtabmap and rtabmap ros

You can find detailed instructions below.

https://github.com/introlab/rtabmap_ros#installation


Tip: Packages will have standard error.  
If you see 
```Policy CMP0074 is not set: find_package uses <PackageName>_ROOT variables```,
you can safely ignore this.  

You should also build the Zed ROS2 wrapper and Zed Ros2 examples packages.

https://github.com/stereolabs/zed-ros2-wrapper


Finally, build the new mapping_module.

`cd rover_ROS_ws/`
`colcon build --symlink`                        ## or
`colcon build --symlink --packages-select mapping_module`  
depending on what you're doing.

Of course, remember to source before running:    
`source rover_ROS_ws/install/setup.bash`  
`source /opt/ros/humble/setup.bash`  


## How to run

`ros2 launch mapping_module new_rtab.launch.py camera_model:=zed2i`


## Output data types
### Core RTAB-Map Outputs

- **Pose / Odometry**
  - `/odom`
  - `/map → odom` TF
- **Point Clouds**
  - Local and global
- **OctoMap** (optional)
- **2D Occupancy Grid**
  - `/map` (`nav_msgs/OccupancyGrid`)

_Data flow:_  
Depth → 3D cloud → raycasting → 2D projection → occupancy map


## Tips

Do not connect the Zed2i camera if not in use.  
If camera is not connecting, restart Ubuntu (or reset your USB bus / PCI connections, if you know what you're doing.)
