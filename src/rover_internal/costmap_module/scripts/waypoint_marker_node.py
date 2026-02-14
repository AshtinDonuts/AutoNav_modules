#!/usr/bin/env python3
"""
Waypoint Marker Node
Publishes visualization markers for the 5 goal waypoints in the navigation sandbox.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math


class WaypointMarkerNode(Node):
    """Node to publish waypoint markers for visualization in RViz."""

    # Define the 5 waypoints for Adelaide Rover Challenge
    WAYPOINTS = [
        {"name": "P1", "x": 3.0, "y": 0.0, "z": 0.0},
        {"name": "P2", "x": 5.0, "y": 3.0, "z": 0.0},
        {"name": "P3", "x": -3.0, "y": -4.0, "z": 0.0},
        {"name": "P4", "x": 2.0, "y": -2.0, "z": 0.0},
        {"name": "P5", "x": 1.0, "y": 4.0, "z": 0.0},
    ]

    def __init__(self):
        super().__init__('waypoint_marker_node')
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Timer to publish markers periodically
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Waypoint Marker Node initialized')

    def publish_markers(self):
        """Publish markers for all waypoints."""
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.WAYPOINTS):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = waypoint["x"]
            marker.pose.position.y = waypoint["y"]
            marker.pose.position.z = waypoint["z"]
            
            # Set orientation (no rotation needed)
            marker.pose.orientation.w = 1.0
            
            # Set scale (0.3m x 0.3m x 0.3m)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Set color (red with full opacity)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # Add text label
            marker.text = waypoint["name"]
            
            marker_array.markers.append(marker)
        
        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
