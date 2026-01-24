import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import pyzed.sl as sl
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class ZedPointCloudNode(Node):
    def __init__(self, svo_filename=None, publish_rate=10.0, topic_name='/zed/points', depth_mode='PERFORMANCE'):
        super().__init__("zed_pointcloud_node")

        self.pub = self.create_publisher(
            PointCloud2,
            topic_name,
            qos_profile_sensor_data
        )

        # Initialize ZED camera
        init_params = sl.InitParameters()
        
        # Set depth mode (default to PERFORMANCE to avoid TensorRT dependency)
        depth_mode_map = {
            'NEURAL': sl.DEPTH_MODE.NEURAL,
            'PERFORMANCE': sl.DEPTH_MODE.PERFORMANCE,
            'QUALITY': sl.DEPTH_MODE.QUALITY,
            'ULTRA': sl.DEPTH_MODE.ULTRA,
        }
        if depth_mode.upper() in depth_mode_map:
            init_params.depth_mode = depth_mode_map[depth_mode.upper()]
            self.get_logger().info(f'Using depth mode: {depth_mode.upper()}')
        else:
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            self.get_logger().warn(f'Unknown depth mode "{depth_mode}", defaulting to PERFORMANCE')
        
        if svo_filename:
            init_params.set_from_svo_file(svo_filename)
            self.get_logger().info(f'Opening SVO file: {svo_filename}')
        
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED camera: {status}')
            self.get_logger().error('Make sure the ZED camera is connected and the ZED SDK is properly installed.')
            raise RuntimeError(f'Failed to open ZED camera: {status}')
        
        self.get_logger().info('ZED camera opened successfully')
        self.get_logger().info(f'Depth mode: {init_params.depth_mode}')

        self.pc = sl.Mat()

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cloud)

    def publish_cloud(self):
        self.zed.retrieve_measure(self.pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)

        arr = self.pc.get_data()  # H x W x 4 float32
        xyz = arr[..., :3].reshape(-1, 3)
        rgba = arr[..., 3].reshape(-1).view(np.uint32)

        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        rgba = rgba[valid]

        points = np.empty(
            xyz.shape[0],
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgba', np.uint32)]
        )
        points['x'] = xyz[:, 0]
        points['y'] = xyz[:, 1]
        points['z'] = xyz[:, 2]
        points['rgba'] = rgba

        header = Header()
        header.frame_id = "zed_left_camera_frame"
        header.stamp = self.get_clock().now().to_msg()

        cloud = pc2.create_cloud(
            header,
            [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
            ],
            points
        )

        cloud.is_dense = False
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ZedPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
