#!/usr/bin/env python3
"""
Costmap Visualizer Node
Subscribes to pointcloud and can provide debugging visualization of the costmap.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


class CostmapVisualizerNode(Node):
    """Node to visualize costmap data from Nav2."""

    def __init__(self):
        super().__init__('costmap_visualizer')
        
        # Subscribe to pointcloud from ZED
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/zed/points',
            self.pointcloud_callback,
            10
        )
        self.pointcloud_subscription  # prevent unused variable warning
        
        # Subscribe to local costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/costmap/costmap',
            self.costmap_callback,
            10
        )
        self.costmap_subscription  # prevent unused variable warning
        
        self.get_logger().info('Costmap Visualizer Node initialized')
        self.pointcloud_count = 0
        self.costmap_count = 0

    def pointcloud_callback(self, msg):
        """Callback for incoming pointcloud data."""
        self.pointcloud_count += 1
        if self.pointcloud_count % 100 == 0:  # Log every 100 messages
            self.get_logger().info(
                f'Received PointCloud2 (count: {self.pointcloud_count}), '
                f'frame: {msg.header.frame_id}, '
                f'width: {msg.width}, height: {msg.height}'
            )

    def costmap_callback(self, msg):
        """Callback for incoming costmap data."""
        self.costmap_count += 1
        if self.costmap_count % 10 == 0:  # Log every 10 messages
            self.get_logger().info(
                f'Received Costmap (count: {self.costmap_count}), '
                f'frame: {msg.header.frame_id}, '
                f'resolution: {msg.info.resolution}, '
                f'width: {msg.info.width}, height: {msg.info.height}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CostmapVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
