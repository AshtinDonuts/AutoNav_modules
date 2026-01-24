"""
ZED VSLAM Node with Live Occupancy Tracking

This node implements a Visual SLAM system using the ZED 2 SDK with:
- Positional tracking with area memory for loop closure
- Live occupancy grid mapping from depth data
- Pose and odometry publishing
- Support for SVO file playback
"""

import numpy as np
import cv2
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped, Twist, TransformStamped
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformBroadcaster

# Try to import ZED SDK
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False
    print("Warning: ZED SDK not available. Please install pyzed.")


class ZEDVSLAMNode(Node):
    """
    ROS2 Node for VSLAM with occupancy grid mapping using ZED camera
    """
    
    def __init__(self, svo_filename=None, area_file_path=None, 
                 grid_resolution=0.05, grid_width=100, grid_height=100,
                 min_height=-0.5, max_height=2.0, publish_rate=10.0):
        super().__init__('zed_vslam_node')
        
        if not ZED_SDK_AVAILABLE:
            self.get_logger().error("ZED SDK not available. Cannot initialize VSLAM.")
            raise RuntimeError("ZED SDK required but not available")
        
        self.bridge = CvBridge()
        self.svo_filename = svo_filename
        self.area_file_path = area_file_path
        self.grid_resolution = grid_resolution
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.min_height = min_height
        self.max_height = max_height
        
        # Occupancy grid parameters
        self.grid_origin_x = -grid_width * grid_resolution / 2.0
        self.grid_origin_y = -grid_height * grid_resolution / 2.0
        
        # Initialize occupancy grid
        self.occupancy_grid = np.full((grid_height, grid_width), -1, dtype=np.int8)
        self.occupancy_counts = np.zeros((grid_height, grid_width), dtype=np.float32)
        
        # Initialize ZED camera
        self.zed = None
        self.init_zed_camera()
        
        # Initialize positional tracking
        self.tracking_enabled = False
        self.enable_positional_tracking()
        
        # Camera pose and tracking state
        self.camera_pose = sl.Pose()
        self.tracking_state = sl.POSITIONAL_TRACKING_STATE.OFF
        self.initial_pose_set = False
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS2 Publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/zed_vslam/occupancy_grid',
            qos_profile
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/zed_vslam/pose',
            10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/zed_vslam/odometry',
            10
        )
        
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/zed_vslam/point_cloud',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/zed_vslam/camera_info',
            10
        )
        
        # Timer for processing frames
        self.timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(self.timer_period, self.process_frame)
        
        # Frame counter for statistics
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('ZED VSLAM node initialized')
        self.get_logger().info(f'Grid resolution: {grid_resolution}m')
        self.get_logger().info(f'Grid size: {grid_width}x{grid_height} cells')
        self.get_logger().info(f'Height filter: {min_height}m to {max_height}m')
    
    def init_zed_camera(self):
        """Initialize ZED camera using ZED SDK"""
        try:
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            
            # Set resolution and depth mode
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use neural depth for better quality
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init_params.sdk_verbose = False
            
            # Handle SVO file input
            if self.svo_filename:
                init_params.set_from_svo_file(self.svo_filename)
                init_params.svo_real_time_mode = True
                self.get_logger().info(f"Using SVO file: {self.svo_filename}")
            
            # Open camera
            status = self.zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {status}")
                raise RuntimeError(f"ZED camera initialization failed: {status}")
            
            # Get camera information
            cam_info = self.zed.get_camera_information()
            self.get_logger().info(f"ZED Camera opened successfully")
            self.get_logger().info(f"Resolution: {cam_info.camera_configuration.resolution.width}x{cam_info.camera_configuration.resolution.height}")
            self.get_logger().info(f"FPS: {cam_info.camera_configuration.fps}")
            
        except Exception as e:
            self.get_logger().error(f"Error initializing ZED camera: {e}")
            raise
    
    def enable_positional_tracking(self):
        """Enable positional tracking with area memory"""
        try:
            tracking_params = sl.PositionalTrackingParameters()
            
            # Enable area memory for loop closure and relocalization
            tracking_params.enable_area_memory = True
            tracking_params.enable_pose_smoothing = True
            tracking_params.enable_imu_fusion = True
            tracking_params.set_floor_as_origin = False
            tracking_params.set_gravity_as_origin = True
            
            # Load area file if provided
            if self.area_file_path:
                tracking_params.area_file_path = self.area_file_path
                self.get_logger().info(f"Loading area memory from: {self.area_file_path}")
            
            # Enable tracking
            status = self.zed.enable_positional_tracking(tracking_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to enable positional tracking: {status}")
                return
            
            self.tracking_enabled = True
            self.get_logger().info("Positional tracking enabled with area memory")
            
        except Exception as e:
            self.get_logger().error(f"Error enabling positional tracking: {e}")
    
    def process_frame(self):
        """Process a frame from ZED camera"""
        if self.zed is None:
            return
        
        # Grab frame
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return
        
        # Get camera pose
        self.tracking_state = self.zed.get_position(
            self.camera_pose, 
            sl.REFERENCE_FRAME.WORLD
        )
        
        # Retrieve images and depth
        image = sl.Mat()   # Calls C constructor (an empty z matrix container)
        depth = sl.Mat()
        point_cloud = sl.Mat()
        
        self.zed.retrieve_image(image, sl.VIEW.LEFT)
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        
        # Publish camera info (only once or periodically)
        if self.frame_count % 30 == 0:
            self.publish_camera_info()
        
        # Update occupancy grid from point cloud
        if self.tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
            self.update_occupancy_grid(point_cloud, self.camera_pose)
            self.publish_pose()
            self.publish_odometry()
            self.publish_tf()
        
        ## HERE WE IMPLEMENT AND/OR PUBLISHING PC / PC-DELTAS / OCCUPANCY GRID
        ## ------------------

        # Publish point cloud (optional, can be disabled for performance)
        # self.publish_point_cloud(point_cloud)

        # self.publish_point_cloud_deltas(point_cloud)
        
        # Publish occupancy grid
        self.publish_occupancy_grid()
        
        self.frame_count += 1
        
        # Log statistics periodically
        if self.frame_count % 100 == 0:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_time).nanoseconds / 1e9
            fps = 100.0 / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f"Tracking state: {self.tracking_state}, "
                f"FPS: {fps:.1f}, "
                f"Frame: {self.frame_count}"
            )
            self.last_time = current_time
    
    def update_occupancy_grid(self, point_cloud, camera_pose):
        """
        Update occupancy grid from point cloud data
        
        Args:
            point_cloud: ZED point cloud (sl.Mat)
            camera_pose: Current camera pose (sl.Pose)
        """
        # Get point cloud data
        pc_data = point_cloud.get_data()
        height, width = pc_data.shape[:2]
        
        # Get camera position in world frame
        camera_translation = camera_pose.get_translation()
        camera_rotation = camera_pose.get_rotation()
        
        # Convert rotation quaternion to rotation matrix
        qx, qy, qz, qw = camera_rotation.get()
        # ZED uses right-handed Y-up coordinate system
        # ROS uses right-handed Z-up, so we need to transform
        
        # Sample points (for performance, process every Nth point)
        step = 2  # Process every 2nd point
        
        for y in range(0, height, step):
            for x in range(0, width, step):
                point = pc_data[y, x]
                
                # Check if point is valid (non-zero and finite)
                if (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2]) or
                    np.isinf(point[0]) or np.isinf(point[1]) or np.isinf(point[2]) or
                    point[2] == 0.0):
                    continue
                
                # Point in camera frame (ZED coordinate system: X-right, Y-down, Z-forward)
                px, py, pz = point[0], point[1], point[2]
                
                # Filter by height (in camera frame, Y is vertical)
                if py < self.min_height or py > self.max_height:
                    continue
                
                # Transform to world frame
                # For 2D occupancy grid, we project onto XZ plane (ground plane)
                # In ZED: X-right, Y-down, Z-forward
                # For 2D map: X-forward, Y-left (ROS convention)
                world_x = camera_translation.get()[0] + px
                world_y = camera_translation.get()[2] + pz  # Use Z as forward
                
                # Convert world coordinates to grid indices
                grid_x = int((world_x - self.grid_origin_x) / self.grid_resolution)
                grid_y = int((world_y - self.grid_origin_y) / self.grid_resolution)
                
                # Check bounds
                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                    # Mark as occupied
                    self.occupancy_counts[grid_y, grid_x] += 1.0
                    
                    # Ray casting: mark free space from camera to point
                    self.raycast_free_space(
                        camera_translation.get()[0],
                        camera_translation.get()[2],
                        world_x, world_y,
                        grid_x, grid_y
                    )
        
        # Update occupancy grid from counts
        # Use a threshold to determine occupancy
        occupied_threshold = 3.0
        self.occupancy_grid = np.where(
            self.occupancy_counts > occupied_threshold,
            100,  # Occupied
            np.where(
                self.occupancy_counts > 0,
                0,  # Free
                -1  # Unknown
            )
        )
    
    def raycast_free_space(self, cam_x, cam_y, point_x, point_y, end_grid_x, end_grid_y):
        """
        Raycast from camera to point, marking free space
        
        Args:
            cam_x, cam_y: Camera position in world coordinates
            point_x, point_y: Point position in world coordinates
            end_grid_x, end_grid_y: End point grid indices
        """
        # Convert camera position to grid
        start_grid_x = int((cam_x - self.grid_origin_x) / self.grid_resolution)
        start_grid_y = int((cam_y - self.grid_origin_y) / self.grid_resolution)
        
        # Bresenham's line algorithm for raycasting
        dx = abs(end_grid_x - start_grid_x)
        dy = abs(end_grid_y - start_grid_y)
        sx = 1 if start_grid_x < end_grid_x else -1
        sy = 1 if start_grid_y < end_grid_y else -1
        err = dx - dy
        
        x, y = start_grid_x, start_grid_y
        
        while True:
            # Check bounds
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                # Only mark as free if not already occupied
                if self.occupancy_counts[y, x] < 1.0:
                    self.occupancy_counts[y, x] = max(0.0, self.occupancy_counts[y, x] - 0.1)
            
            if x == end_grid_x and y == end_grid_y:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def publish_occupancy_grid(self):
        """Publish occupancy grid as ROS message"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten grid and convert to list
        # Occupancy grid uses row-major order
        grid_flat = self.occupancy_grid.flatten()
        msg.data = grid_flat.tolist()
        
        self.occupancy_grid_pub.publish(msg)
    
    def publish_pose(self):
        """Publish camera pose"""
        if self.tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        translation = self.camera_pose.get_translation()
        rotation = self.camera_pose.get_rotation()
        
        pose_msg.pose.position.x = float(translation.get()[0])
        pose_msg.pose.position.y = float(translation.get()[1])
        pose_msg.pose.position.z = float(translation.get()[2])
        
        pose_msg.pose.orientation.x = float(rotation.get()[0])
        pose_msg.pose.orientation.y = float(rotation.get()[1])
        pose_msg.pose.orientation.z = float(rotation.get()[2])
        pose_msg.pose.orientation.w = float(rotation.get()[3])
        
        self.pose_pub.publish(pose_msg)
    
    def publish_odometry(self):
        """Publish odometry message"""
        if self.tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return
        
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'zed_camera_center'
        
        translation = self.camera_pose.get_translation()
        rotation = self.camera_pose.get_rotation()
        
        odom_msg.pose.pose.position.x = float(translation.get()[0])
        odom_msg.pose.pose.position.y = float(translation.get()[1])
        odom_msg.pose.pose.position.z = float(translation.get()[2])
        
        odom_msg.pose.pose.orientation.x = float(rotation.get()[0])
        odom_msg.pose.pose.orientation.y = float(rotation.get()[1])
        odom_msg.pose.pose.orientation.z = float(rotation.get()[2])
        odom_msg.pose.pose.orientation.w = float(rotation.get()[3])
        
        # Set covariance (placeholder values)
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[14] = 0.01  # z
        odom_msg.pose.covariance[21] = 0.01  # roll
        odom_msg.pose.covariance[28] = 0.01  # pitch
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        self.odom_pub.publish(odom_msg)
    
    def publish_tf(self):
        """Publish transform from map to camera"""
        if self.tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'zed_camera_center'
        
        translation = self.camera_pose.get_translation()
        rotation = self.camera_pose.get_rotation()
        
        t.transform.translation.x = float(translation.get()[0])
        t.transform.translation.y = float(translation.get()[1])
        t.transform.translation.z = float(translation.get()[2])
        
        t.transform.rotation.x = float(rotation.get()[0])
        t.transform.rotation.y = float(rotation.get()[1])
        t.transform.rotation.z = float(rotation.get()[2])
        t.transform.rotation.w = float(rotation.get()[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_camera_info(self):
        """Publish camera info"""
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters.left_cam
        
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'zed_camera_center'
        
        info_msg.width = calib.image_size.width
        info_msg.height = calib.image_size.height
        
        # Intrinsic matrix
        info_msg.k[0] = calib.fx
        info_msg.k[2] = calib.cx
        info_msg.k[4] = calib.fy
        info_msg.k[5] = calib.cy
        info_msg.k[8] = 1.0
        
        # Distortion model
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d[0] = calib.disto[0]
        info_msg.d[1] = calib.disto[1]
        info_msg.d[2] = calib.disto[2]
        info_msg.d[3] = calib.disto[3]
        info_msg.d[4] = calib.disto[4]
        
        self.camera_info_pub.publish(info_msg)
    
    def publish_point_cloud(self, point_cloud):
        """Publish point cloud (optional, for visualization)"""
        # An implementation is found in zed_3dpc_node.py
        # Not yet implemented in this class.
        pass
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.zed is not None:
            # Save area memory if enabled
            if self.tracking_enabled and self.area_file_path:
                try:
                    self.zed.save_area_map(self.area_file_path)
                    self.get_logger().info(f"Area memory saved to: {self.area_file_path}")
                except Exception as e:
                    self.get_logger().warn(f"Could not save area memory: {e}")
            
            if self.tracking_enabled:
                self.zed.disable_positional_tracking()
            
            self.zed.close()
        
        super().destroy_node()
