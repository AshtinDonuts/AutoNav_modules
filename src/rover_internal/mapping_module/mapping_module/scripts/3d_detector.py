#!/usr/bin/env python3
"""
3D Object Detection Node using ZED Camera and Mask R-CNN

This node integrates the zed-pytorch 3D Mask R-CNN detector with ROS2.
It subscribes to ZED camera image and depth topics, performs object detection,
and publishes 3D object detections with positions, bounding boxes, and masks.
"""

import sys
import os
import argparse
import numpy as np
import cv2
import torch
import torchvision.transforms as T

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# Try to import zed-pytorch components
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False
    print("Warning: ZED SDK not available. Some features may be limited.")

# Try to import maskrcnn-benchmark
MASK_RCNN_AVAILABLE = False
try:
    from maskrcnn_benchmark.config import cfg
    from maskrcnn_benchmark.engine.predictor import COCODemo
    MASK_RCNN_AVAILABLE = True
except ImportError:
    # If maskrcnn-benchmark is installed in a non-standard location,
    # add it to sys.path before importing
    pass


class ZED3DDetector(Node):
    """
    ROS2 Node for 3D object detection using ZED camera and Mask R-CNN
    """
    
    def __init__(self, config_file=None, min_image_size=256, device='cuda', 
                 show_mask_heatmaps=False, svo_filename=None):
        super().__init__('zed_3d_detector')
        
        self.bridge = CvBridge()
        self.device = device if torch.cuda.is_available() else 'cpu'
        self.min_image_size = min_image_size
        self.show_mask_heatmaps = show_mask_heatmaps
        
        # Initialize ZED camera if SDK is available
        self.zed = None
        self.use_zed_sdk = False
        if ZED_SDK_AVAILABLE and svo_filename is None:
            self.init_zed_camera()
        
        # Initialize Mask R-CNN model
        self.predictor = None
        if MASK_RCNN_AVAILABLE and config_file:
            self.init_maskrcnn(config_file)
        elif not MASK_RCNN_AVAILABLE:
            self.get_logger().warn("Mask R-CNN not available. Running in depth-only mode.")
        
        # Camera intrinsics (will be updated from CameraInfo)
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Frame buffers
        self.latest_image = None
        self.latest_depth = None
        self.latest_image_time = None
        self.latest_depth_time = None
        
        # ROS2 Subscribers
        if not self.use_zed_sdk:
            # Subscribe to ZED ROS wrapper topics
            self.image_sub = self.create_subscription(
                Image,
                '/zed/zed_node/rgb/image_rect_color',
                self.image_callback,
                10
            )
            
            self.depth_sub = self.create_subscription(
                Image,
                '/zed/zed_node/depth/depth_registered',
                self.depth_callback,
                10
            )
            
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/zed/zed_node/rgb/camera_info',
                self.camera_info_callback,
                10
            )
        
        # ROS2 Publishers
        self.detections_pub = self.create_publisher(
            MarkerArray,
            '/zed_3d_detector/detections',
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            '/zed_3d_detector/annotated_image',
            10
        )
        
        # Timer for processing (if using ZED SDK directly)
        if self.use_zed_sdk:
            self.timer = self.create_timer(0.033, self.process_zed_frame)  # ~30 FPS
        
        self.get_logger().info('ZED 3D Detector node initialized')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Min image size: {self.min_image_size}')
    
    def init_zed_camera(self):
        """Initialize ZED camera using ZED SDK"""
        try:
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.METER
            
            status = self.zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {status}")
                self.zed = None
                return
            
            self.use_zed_sdk = True
            self.get_logger().info("ZED camera initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Error initializing ZED camera: {e}")
            self.zed = None
    
    def init_maskrcnn(self, config_file):
        """Initialize Mask R-CNN model"""
        try:
            # Load config
            cfg.merge_from_file(config_file)
            cfg.merge_from_list(["MODEL.DEVICE", self.device])
            cfg.merge_from_list(["MODEL.WEIGHT", ""])  # Will download automatically
            
            # Create predictor
            self.predictor = COCODemo(
                cfg,
                min_image_size=self.min_image_size,
                confidence_threshold=0.5,
                show_mask_heatmaps=self.show_mask_heatmaps
            )
            
            self.get_logger().info(f"Mask R-CNN model loaded from {config_file}")
        except Exception as e:
            self.get_logger().error(f"Error loading Mask R-CNN model: {e}")
            self.predictor = None
    
    def camera_info_callback(self, msg):
        """Update camera intrinsics from CameraInfo message"""
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().debug(f"Camera intrinsics updated: fx={self.fx}, fy={self.fy}")
    
    def image_callback(self, msg):
        """Callback for RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            self.latest_image_time = msg.header.stamp
            
            # Process if we have both image and depth
            if self.latest_depth is not None:
                self.process_detection()
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.latest_depth = depth_image
            self.latest_depth_time = msg.header.stamp
            
            # Process if we have both image and depth
            if self.latest_image is not None:
                self.process_detection()
        except Exception as e:
            self.get_logger().error(f"Error in depth callback: {e}")
    
    def process_zed_frame(self):
        """Process frame from ZED SDK directly"""
        if self.zed is None:
            return
        
        # Grab frame
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve images
            image = sl.Mat()
            depth = sl.Mat()
            
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            
            # Convert to numpy arrays
            cv_image = image.get_data()
            depth_array = depth.get_data()
            
            self.latest_image = cv_image
            self.latest_depth = depth_array
            
            # Get camera parameters
            cam_info = self.zed.get_camera_information()
            self.fx = cam_info.camera_configuration.calibration_parameters.left_cam.fx
            self.fy = cam_info.camera_configuration.calibration_parameters.left_cam.fy
            self.cx = cam_info.camera_configuration.calibration_parameters.left_cam.cx
            self.cy = cam_info.camera_configuration.calibration_parameters.left_cam.cy
            
            self.process_detection()
    
    def process_detection(self):
        """Main detection processing function"""
        if self.latest_image is None or self.latest_depth is None:
            return
        
        if self.fx is None or self.fy is None:
            self.get_logger().warn("Camera intrinsics not available yet")
            return
        
        # Run Mask R-CNN detection if available
        detections = []
        annotated_image = self.latest_image.copy()
        
        if self.predictor is not None:
            try:
                # Run inference :
                # =====================
                # CORE interfacing code
                predictions = self.predictor.compute_prediction(self.latest_image)
                top_predictions = self.predictor.select_top_predictions(predictions)
                
                # Extract detections
                boxes = top_predictions.bbox.cpu().numpy()
                labels = top_predictions.get_field("labels").cpu().numpy()
                scores = top_predictions.get_field("scores").cpu().numpy()
                
                # Get masks if available
                masks = None
                if top_predictions.has_field("mask"):
                    masks = top_predictions.get_field("mask")
                
                # Process each detection
                for i in range(len(boxes)):
                    bbox = boxes[i]
                    label = labels[i]
                    score = scores[i]
                    
                    # Calculate 3D position
                    x1, y1, x2, y2 = bbox
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    # Get depth at centroid
                    depth_value = self.latest_depth[cy, cx] if cy < self.latest_depth.shape[0] and cx < self.latest_depth.shape[1] else 0
                    
                    if depth_value > 0 and not np.isnan(depth_value) and not np.isinf(depth_value):
                        # Backproject to 3D
                        X = (cx - self.cx) * depth_value / self.fx
                        Y = (cy - self.cy) * depth_value / self.fy
                        Z = depth_value
                        
                        detections.append({
                            'bbox': bbox,
                            'label': label,
                            'score': score,
                            'position_3d': [X, Y, Z],
                            'mask': masks[i] if masks is not None else None
                        })
                        
                        # Draw on annotated image
                        cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(annotated_image, f"{label}: {score:.2f}", 
                                   (int(x1), int(y1) - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.circle(annotated_image, (cx, cy), 5, (0, 0, 255), -1)
            
            except Exception as e:
                self.get_logger().error(f"Error in Mask R-CNN inference: {e}")
        
        # Publish detections as markers
        if detections:
            self.publish_detections(detections)
        
        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            annotated_msg.header.frame_id = 'zed_camera_center'
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")
    
    def publish_detections(self, detections):
        """Publish detections as ROS2 MarkerArray"""
        marker_array = MarkerArray()
        
        for idx, det in enumerate(detections):
            # Create marker for 3D position
            marker = Marker()
            marker.header.frame_id = 'zed_camera_center'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            pos = det['position_3d']
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            
            # Set scale and color
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color based on label
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.id = idx + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = marker.pose.position
            text_marker.pose.position.z += 0.2
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.1
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"Label: {det['label']}, Score: {det['score']:.2f}"
            
            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)
        
        self.detections_pub.publish(marker_array)


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='ZED 3D Object Detection Node')
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to Mask R-CNN config file')
    parser.add_argument('--min-image-size', type=int, default=256,
                       help='Minimum image size for inference')
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'],
                       help='Device to run inference on')
    parser.add_argument('--show-mask-heatmaps', action='store_true',
                       help='Show mask heatmaps')
    parser.add_argument('--svo-filename', type=str, default=None,
                       help='Path to SVO file to replay')
    
    # Parse ROS2 args
    rclpy.init(args=args)
    
    # Get node args
    node_args = rclpy.utilities.remove_ros_args(args) if args else sys.argv[1:]
    parsed_args = parser.parse_args(node_args)
    
    # Create node
    node = ZED3DDetector(
        config_file=parsed_args.config_file,
        min_image_size=parsed_args.min_image_size,
        device=parsed_args.device,
        show_mask_heatmaps=parsed_args.show_mask_heatmaps,
        svo_filename=parsed_args.svo_filename
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
