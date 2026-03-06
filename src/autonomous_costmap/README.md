# pcd_to_height_costmap Spec (For Planning Integration)

## 1. Purpose
This node converts input `PointCloud2` into two `OccupancyGrid` maps:
- `/height_map`: normalized per-cell height preview (debug map)
- `/height_traversability_costmap`: slope-derived traversability costmap (planning map)

Use `/height_traversability_costmap` for path planning.

## 2. Topics
### Input
- `input_pointcloud_topic` (default currently used in tests: `/gazebo_ros_velodyne/out`)
- Type: `sensor_msgs/msg/PointCloud2`

### Outputs
- `/height_map`
- `/height_traversability_costmap`
- Type: `nav_msgs/msg/OccupancyGrid`

## 3. Value Semantics (Important)
### `/height_map`
- Meaning: normalized height value, not traversability
- Encodes local mean `z` into `0..100`
- `-1` = unknown/no valid data in cell

### `/height_traversability_costmap`
- Meaning: traversability cost from local slope
- `0` = easiest / most traversable
- `1..99` = increasing traversal difficulty
- `100` = non-traversable (lethal)
- unknown cells are `100` when `unknown_as_obstacle=True`, otherwise `-1`

Do not infer semantics from RViz color names. Use numeric values.

## 4. Core Algorithm
For each point cloud frame:
1. Filter points by `min_z`, `max_z`, `max_range`
2. Transform points from source frame to `frame_id` using TF
3. Bin points into 2D grid cells (`resolution`, `width_m`, `height_m`)
4. Compute per-cell mean height (requires `min_points_per_cell`)
5. Build `/height_map` from normalized mean height
6. Build traversability map from max local slope (8-neighborhood):
   - slope <= `free_slope_deg` -> cost 0
   - slope >= `lethal_slope_deg` -> cost 100
   - between them -> linear interpolation to `1..99`

## 5. Why Arc/Wave Patterns Appear
The arc-like patterns are expected from LiDAR angular sampling geometry.
- LiDAR samples by angle, not equal distance spacing.
- Near range: samples look denser.
- Far range: arc spacing becomes larger.
- At cliff edges or no-return regions, there are no points, so cells stay unknown.

These patterns are data sampling artifacts, not planner commands by themselves.

## 6. Parameter Tuning Guide
### `min_z`, `max_z`
Use to keep useful terrain points and remove outliers.
- Too wide: more noise and unstable costs
- Too narrow: lose valid terrain/obstacle points

Recommended starting range in this Gazebo setup:
- `min_z: -0.5 to -1.0`
- `max_z: 1.5 to 2.0`

### `max_range`
Controls how far points are accepted.
- Larger: more coverage but more sparse/noisy arcs
- Smaller: cleaner local map but shorter horizon

Recommended start:
- `8.0 to 10.0` for stable local planning

### `min_points_per_cell`
- Larger value: cleaner, but more unknown cells
- Smaller value: denser map, but noisier

Recommended start:
- `2` (balanced)

### `free_slope_deg`, `lethal_slope_deg`
Defines traversability slope bands.
- Lower thresholds = more conservative
- Higher thresholds = more permissive

Recommended start:
- `free_slope_deg: 5`
- `lethal_slope_deg: 25`

## 7. Planning Integration Contract (For Seng)
Planner should subscribe to:
- `/height_traversability_costmap`

Suggested interpretation:
- `0..30`: low cost (preferred)
- `31..69`: moderate risk (allowed when necessary)
- `70..99`: high risk (penalize strongly)
- `100`: blocked / non-traversable
- `-1` (if used): unknown (policy dependent)

If `unknown_as_obstacle=True`, unknown is already treated as blocked (`100`).

## 8. Minimal Runtime Commands
### Run costmap node
```bash
cd /home/unixuser/AutoNav_modules
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run autonomous_costmap pcd_to_height_costmap --ros-args \
  -p input_pointcloud_topic:=/gazebo_ros_velodyne/out \
  -p frame_id:=odom
```

### RViz
- Fixed Frame: `odom`
- Add `Map` display for `/height_map`
- Add `Map` display for `/height_traversability_costmap`

## 9. Notes
- `/height_map` is debug visualization.
- `/height_traversability_costmap` is the planning input.
- Color in RViz is display-dependent; numeric value is ground truth.
