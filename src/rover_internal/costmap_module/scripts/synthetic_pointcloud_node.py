#!/usr/bin/env python3
import math
import struct
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class SyntheticPointCloudNode(Node):
    def __init__(self):
        super().__init__('synthetic_pointcloud_node')
        # Parameters
        self.declare_parameter('frame_id', 'zed_left_camera_frame')
        self.declare_parameter('topic', '/zed/points')
        self.declare_parameter('rate_hz', 5.0)  # Reduced from 10 to 5 Hz
        self.declare_parameter('range_m', 8.0)
        self.declare_parameter('num_points', 1000)  # Reduced from 2000 to 1000
        self.declare_parameter('pattern', 'fan')  # 'fan' or 'wall'

        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.range_m = float(self.get_parameter('range_m').value)
        self.num_points = int(self.get_parameter('num_points').value)
        self.pattern = self.get_parameter('pattern').value

        self.pub = self.create_publisher(PointCloud2, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_cloud)
        self.phase = 0.0
        self.get_logger().info(f'Publishing synthetic pointcloud on {self.topic} at {self.rate_hz} Hz, frame={self.frame_id}')

    def publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # Generate points in sensor frame
        points = []
        if self.pattern == 'fan':
            # Fan pattern sweeping left-right over time
            sweep = math.sin(self.phase) * 0.8  # radians
            for i in range(self.num_points):
                # angle relative to sensor forward
                a = sweep + (random.random() - 0.5) * 0.6
                r = random.random() * self.range_m
                x = r * math.cos(a)
                y = r * math.sin(a)
                z = (random.random() - 0.5) * 0.1  # thin band
                # Keep points in front of sensor
                if x > 0.2:
                    points.append((x, y, z))
        elif self.pattern == 'wall':
            # Wall pattern: vertical plane ahead
            for i in range(self.num_points):
                x = 3.0 + (random.random() - 0.5) * 0.1
                y = (random.random() - 0.5) * 4.0
                z = (random.random() - 0.5) * 0.5
                points.append((x, y, z))
        elif self.pattern == 'obstacles':
            # Multiple obstacles: front wall + left/right obstacles
            # Front wall at 3m
            num_per_obj = self.num_points // 3
            for i in range(num_per_obj):
                x = 3.0 + (random.random() - 0.5) * 0.2
                y = (random.random() - 0.5) * 2.0
                z = (random.random() - 0.5) * 0.4
                points.append((x, y, z))
            # Left obstacle at 1.5m
            for i in range(num_per_obj):
                x = 1.5 + (random.random() - 0.5) * 0.3
                y = 1.0 + (random.random() - 0.5) * 0.3
                z = (random.random() - 0.5) * 0.4
                points.append((x, y, z))
            # Right obstacle at 1.5m
            for i in range(num_per_obj):
                x = 1.5 + (random.random() - 0.5) * 0.3
                y = -1.0 + (random.random() - 0.5) * 0.3
                z = (random.random() - 0.5) * 0.4
                points.append((x, y, z))

        self.phase += 0.1
        msg = self.create_pointcloud2(header, points)
        self.pub.publish(msg)

    @staticmethod
    def create_pointcloud2(header: Header, points):
        # Define fields x,y,z as float32
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12  # 3 * float32
        data = bytearray()
        for x, y, z in points:
            data += struct.pack('<fff', float(x), float(y), float(z))
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * cloud.width
        cloud.data = bytes(data)
        cloud.is_dense = True
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticPointCloudNode()
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
