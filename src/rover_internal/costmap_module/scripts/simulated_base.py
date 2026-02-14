#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimulatedBase(Node):
    def __init__(self):
        super().__init__('simulated_base')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.br = TransformBroadcaster(self)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        self.get_logger().info(f'Simulated base listening on {self.cmd_vel_topic}')

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        # Integrate pose in the robot frame (vx, vy) rotated by yaw into map/odom plane
        dx = self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)
        dy = self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)
        self.x += dx * dt
        self.y += dy * dt
        self.yaw += self.wz * dt

        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.br.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedBase()
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
