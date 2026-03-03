#!/usr/bin/env python3
"""
Convert PointCloud2 into a traversability costmap.

This node builds a 2.5D grid from incoming point clouds:
1) Per-cell mean height (for debugging and visualization)
2) Per-cell traversability cost derived from local slope

Outputs:
- /height_map (nav_msgs/OccupancyGrid): normalized height preview [-1 unknown, 0..100]
- /height_traversability_costmap (nav_msgs/OccupancyGrid): planner cost [-1 unknown, 0..100]

"""

import math
from typing import List

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class PcdToHeightCostmapNode(Node):
    def __init__(self) -> None:
        super().__init__('pcd_to_height_costmap')

        # Input / output topics
        # IMPORTANT: Change it here if your ZED/PCD source topic is different.
        self.declare_parameter('input_pointcloud_topic', '/zed/zed_node/point_cloud/cloud_registered')
        # Output topic: normalized height preview (debug visualization map)
        self.declare_parameter('height_map_topic', '/height_map')
        # Output topic: planner-facing traversability costmap (0..100 / -1 unknown)
        self.declare_parameter('traversability_topic', '/height_traversability_costmap')

        # Grid geometry (local rolling window in sensor frame by default)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('resolution', 0.10)  # meters per cell
        self.declare_parameter('width_m', 20.0)     # map width in meters
        self.declare_parameter('height_m', 20.0)    # map height in meters

        # Point filtering
        # Default rationale:
        # - min_z=-1.0, max_z=2.0 keeps ground/ramps and typical robot-height obstacles,
        #   while removing obvious outliers above the useful terrain band.
        # - max_range=15.0 limits far sparse points that are less useful for local decisions.
        # - min_points_per_cell=2 avoids treating single-point noise as reliable height.
        self.declare_parameter('min_z', -1.0)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_points_per_cell', 2)

        # Height map normalization for debugging output
        self.declare_parameter('height_norm_min', -1.0)
        self.declare_parameter('height_norm_max', 1.5)

        # Traversability rules
        # IMPORTANT: Tune these two slope thresholds to change what terrain is considered traversable.
        # - free_slope_deg: slopes at/below this are treated as free (cost ~ 0)
        # - lethal_slope_deg: slopes at/above this are treated as non-traversable (cost = 100)
        self.declare_parameter('free_slope_deg', 5.0)
        self.declare_parameter('lethal_slope_deg', 25.0)
        self.declare_parameter('unknown_as_obstacle', True)

        # Load parameter values
        self.input_topic = str(self.get_parameter('input_pointcloud_topic').value)
        self.height_map_topic = str(self.get_parameter('height_map_topic').value)
        self.traversability_topic = str(self.get_parameter('traversability_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)

        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_points = int(self.get_parameter('min_points_per_cell').value)

        self.height_norm_min = float(self.get_parameter('height_norm_min').value)
        self.height_norm_max = float(self.get_parameter('height_norm_max').value)

        self.free_slope_deg = float(self.get_parameter('free_slope_deg').value)
        self.lethal_slope_deg = float(self.get_parameter('lethal_slope_deg').value)
        self.unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)

        # Discrete grid dimensions
        self.width_cells = max(1, int(round(self.width_m / self.resolution)))
        self.height_cells = max(1, int(round(self.height_m / self.resolution)))
        self.cell_count = self.width_cells * self.height_cells

        # Centered local window around origin of frame_id.
        # Example: width=20m, height=20m -> origin=(-10,-10), so map spans [-10,+10] in both axes.
        # This keeps (0,0) near map center instead of at the bottom-left corner.
        self.origin_x = -0.5 * self.width_m
        self.origin_y = -0.5 * self.height_m

        # Publishers
        self.height_pub = self.create_publisher(OccupancyGrid, self.height_map_topic, 10)
        self.trav_pub = self.create_publisher(OccupancyGrid, self.traversability_topic, 10)

        # Subscriber
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, 10)

        self.get_logger().info(
            f'pcd_to_height_costmap ready | input={self.input_topic} | '
            f'height={self.height_map_topic} | traversability={self.traversability_topic}'
        )

    # Main processing callback for each incoming PointCloud2 frame.
    # What it does:
    # 1) Filters raw points by z-range and distance.
    # 2) Bins valid points into 2D grid cells and computes per-cell mean height.
    # 3) Publishes two OccupancyGrid outputs:
    #    - /height_map (normalized height preview)
    #    - /height_traversability_costmap (planner cost derived from slope)
    def cloud_callback(self, msg: PointCloud2) -> None:
        # Per-cell accumulators for mean height
        sums = [0.0] * self.cell_count
        counts = [0] * self.cell_count

        max_range_sq = self.max_range * self.max_range

        # Read XYZ points from PointCloud2 and bin into grid cells.
        # z from PointCloud2 is treated as cell height information.
        for x, y, z in read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            # Filter points by height and distance to reduce noise and focus on relevant terrain.
            if z < self.min_z or z > self.max_z:
                continue
            # Filter points that are too far away to be relevant for local navigation.
            if (x * x + y * y) > max_range_sq:
                continue

            # Convert (x,y) to grid cell indices (matrix_x, matrix_y).
            # Points outside the grid are ignored.
            matrix_x = int((x - self.origin_x) / self.resolution)
            matrix_y = int((y - self.origin_y) / self.resolution)

            if matrix_x < 0 or matrix_y < 0 or matrix_x >= self.width_cells or matrix_y >= self.height_cells:
                continue

            idx = matrix_y * self.width_cells + matrix_x
            # Accumulate point height and count for this cell so we can compute
            # a stable per-cell mean height later.
            sums[idx] += float(z)
            counts[idx] += 1

        # Mean height map (None means unknown cell)
        mean_heights: List[float] = [math.nan] * self.cell_count
        for idx in range(self.cell_count):
            # Only compute mean height when enough points are present in a cell.
            # Otherwise keep NaN (unknown) to avoid noisy or unreliable estimates.
            if counts[idx] >= self.min_points:
                mean_heights[idx] = sums[idx] / counts[idx]

        self.publish_height_map(msg, mean_heights)
        self.publish_traversability_map(msg, mean_heights)

    def publish_height_map(self, cloud_msg: PointCloud2, mean_heights: List[float]) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = cloud_msg.header.stamp
        grid.header.frame_id = self.frame_id

        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        hmin = self.height_norm_min
        hmax = self.height_norm_max
        span = max(1e-6, hmax - hmin)

        data: List[int] = [-1] * self.cell_count
        for idx, h in enumerate(mean_heights):
            if math.isnan(h):
                data[idx] = -1
                continue

            # Normalize height to 0..100 for quick RViz debugging
            normalized = int(round(((h - hmin) / span) * 100.0))
            data[idx] = max(0, min(100, normalized))

        grid.data = data
        self.height_pub.publish(grid)

    def publish_traversability_map(self, cloud_msg: PointCloud2, mean_heights: List[float]) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = cloud_msg.header.stamp
        grid.header.frame_id = self.frame_id

        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        data: List[int] = [-1] * self.cell_count
        free_slope = self.free_slope_deg
        lethal_slope = max(self.lethal_slope_deg, free_slope + 1e-6)

        neighbors = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),           (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        ]

        for my in range(self.height_cells):
            for mx in range(self.width_cells):
                idx = my * self.width_cells + mx
                h = mean_heights[idx]

                if math.isnan(h):
                    data[idx] = 100 if self.unknown_as_obstacle else -1
                    continue

                # Estimate max local slope against 8-neighborhood.
                # Use neighboring cell height differences (delta z) to infer terrain slope.
                max_slope_deg = 0.0
                for dx, dy in neighbors:
                    nx = mx + dx
                    ny = my + dy
                    if nx < 0 or ny < 0 or nx >= self.width_cells or ny >= self.height_cells:
                        continue

                    nidx = ny * self.width_cells + nx
                    nh = mean_heights[nidx]
                    if math.isnan(nh):
                        continue

                    # dz is height difference between two adjacent cells.
                    dz = abs(h - nh)
                    dist = self.resolution * (math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0)
                    # slope_deg = atan(dz / horizontal_distance), converted to degrees.
                    # This is the key step that converts height (z) into traversability.
                    slope_deg = math.degrees(math.atan2(dz, dist))
                    if slope_deg > max_slope_deg:
                        max_slope_deg = slope_deg

                # Convert slope to planner cost (0..100):
                # <= free_slope -> free (0)
                # >= lethal_slope -> non-traversable (100)
                # in between -> linearly interpolated cost
                if max_slope_deg <= free_slope:
                    cost = 0
                elif max_slope_deg >= lethal_slope:
                    cost = 100
                else:
                    ratio = (max_slope_deg - free_slope) / (lethal_slope - free_slope)
                    cost = int(round(ratio * 99.0))

                data[idx] = max(0, min(100, cost))

        grid.data = data
        self.trav_pub.publish(grid)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcdToHeightCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
