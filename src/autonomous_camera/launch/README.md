tldr: use new_rtab.launch.py for EVERYTHING

In detail:

`new_rtab.launch.py` can give you the following output:

## RTAB-Map Core Outputs

- **Pose / Odometry:**  
  - `/odom`  
  - `/map → odom` (TF)

- **3D Point Clouds:**  
  - Local & Global

- **3D OctoMap:**  
  - _(optional)_

- **2D Occupancy Grid:**  
  - `/map` (`nav_msgs/OccupancyGrid`)

## Overview

### 1. ZED Camera Node (`zed_node`)
- **Purpose:** Captures camera data from the ZED2i camera
- **Publishes:**
  - RGB images: `/zed/zed_node/rgb/color/rect/image`
  - Depth images: `/zed/zed_node/depth/depth_registered`
  - Camera info: `/zed/zed_node/rgb/color/rect/camera_info`
  - IMU data: `/zed/zed_node/imu/data`
  - Odometry (optional): `/zed/zed_node/odom`

---

### 2. RGBD Sync Node (`rgbd_sync` from `rtabmap_sync`)
- **Purpose:** Synchronizes RGB + Depth + Camera Info streams with approximate timestamp matching
- **Subscribes to:** Individual RGB, Depth, and Camera Info topics
- **Publishes:** `/rgbd_image` (synchronized RGBD message)
- **Key parameter:** `approx_sync_max_interval: 0.02` _(20ms tolerance)_

---

### 3. Visual Odometry Node (`rgbd_odometry` from `rtabmap_odom`)
- **Purpose:** Estimates camera motion frame-to-frame using visual features
- **Condition:** Only runs if `use_zed_odometry:=false` (default in your config)
- **Publishes:** `/odom` (pose estimates)
- **Needs:** IMU data (`wait_imu_to_init:=True`)

---

### 4. RTAB-Map SLAM Node (`rtabmap` from `rtabmap_slam`)
- **Purpose:** Main SLAM engine – builds map, detects loop closures, optimizes trajectory
- **Subscribes to:** RGBD images, odometry, IMU, _(optional)_ laser scans
- **Publishes:** Map data, corrected poses, occupancy grids

### 5. Visualization Nodes

- **`rtabmap_viz`:** RTAB-Map’s 3D visualization GUI


---

Alternatively you can use the `zed_3dpc` launch file, which gives you a simple Nav2 compatible PointCloud2 message type. 

The older launch files folder contains pretty much deprecated stuff I just wanted to record.  
There is little use for that stuff.

