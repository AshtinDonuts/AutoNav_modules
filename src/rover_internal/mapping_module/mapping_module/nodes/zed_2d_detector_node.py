"""
2D Object Detection Node using ZED Camera with Custom Detector

This node integrates a 2D bounding box detector (e.g., YOLO) with the ZED SDK's
custom object detection API. The 2D detections are ingested into the ZED SDK,
which computes 3D positions, bounding boxes, and tracks objects over time.

Based on: https://www.stereolabs.com/docs/object-detection/custom-od
"""

import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# Try to import ZED SDK
try:
    import pyzed.sl as sl
    ZED_SDK_AVAILABLE = True
except ImportError:
    ZED_SDK_AVAILABLE = False
    print("Warning: ZED SDK not available. ZED SDK is required for 2D detector.")


class SimpleYOLODetector:
    """
    Simple YOLO-style detector wrapper
    This is a placeholder - replace with your actual YOLO implementation
    """
    
    def __init__(self, model_path=None, confidence_threshold=0.5, device='cuda', logger=None):
        self.confidence_threshold = confidence_threshold
        self.device = device if torch.cuda.is_available() else 'cpu'
        self.model = None
        self.logger = logger
        
        # Try to load YOLO model if available
        try:
            # Example: Using ultralytics YOLO
            from ultralytics import YOLO
            if model_path:
                self.model = YOLO(model_path)
            else:
                # Use default YOLOv8 model
                self.model = YOLO('yolov8n.pt')
            if self.logger:
                self.logger.info(f"YOLO model loaded: {model_path or 'yolov8n.pt'}")
        except ImportError:
            if self.logger:
                self.logger.warn("ultralytics YOLO not available. Using dummy detector.")
            self.model = None
    
    def detect(self, image):
        """
        Run detection on image
        
        Returns:
            list of detections, each with:
            - bbox: [x1, y1, x2, y2] in image coordinates
            - conf: confidence score
            - class_id: class ID
        """
        if self.model is None:
            # Dummy detector for testing
            return []
        
        try:
            results = self.model(image, conf=self.confidence_threshold)
            detections = []
            
            for result in results:
                boxes = result.boxes
                for i in range(len(boxes)):
                    box = boxes[i]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    
                    detections.append({
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'conf': conf,
                        'class_id': cls
                    })
            
            return detections
        except Exception as e:
            print(f"Error in YOLO detection: {e}")
            return []


class ZED2DDetector(Node):
    """
    ROS2 Node for 2D object detection using custom detector with ZED SDK
    
    This node uses a 2D bounding box detector (e.g., YOLO) to detect objects,
    then ingests the detections into the ZED SDK which computes 3D positions
    and tracks objects over time.
    """
    
    def __init__(self, model_path=None, confidence_threshold=0.5, device='cuda',
                 enable_tracking=True, svo_filename=None, image_resolution=None):
        super().__init__('zed_2d_detector')
        
        if not ZED_SDK_AVAILABLE:
            self.get_logger().error("ZED SDK is required but not available!")
            raise RuntimeError("ZED SDK not available")
        
        self.bridge = CvBridge()
        self.device = device if torch.cuda.is_available() else 'cpu'
        self.enable_tracking = enable_tracking
        
        # Set default image resolution
        if image_resolution is None:
            self.image_resolution = sl.RESOLUTION.HD720
        else:
            self.image_resolution = image_resolution
        
        # Initialize 2D detector
        self.detector = SimpleYOLODetector(
            model_path=model_path,
            confidence_threshold=confidence_threshold,
            device=self.device,
            logger=self.get_logger()
        )
        
        # Initialize ZED camera
        self.zed = None
        self.init_zed_camera(svo_filename)
        
        # Object detection parameters
        self.detection_parameters = None
        self.detection_runtime_parameters = None
        self.init_object_detection()
        
        # Frame storage
        self.left_image = sl.Mat()
        self.detection_image = sl.Mat()
        
        # ROS2 Publishers
        self.detections_pub = self.create_publisher(
            MarkerArray,
            '/zed_2d_detector/detections',
            10
        )
        
        self.image_pub = self.create_publisher(
            Image,
            '/zed_2d_detector/annotated_image',
            10
        )
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 FPS
        
        self.get_logger().info('ZED 2D Detector node initialized')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Tracking enabled: {self.enable_tracking}')
    
    def init_zed_camera(self, svo_filename=None):
        """Initialize ZED camera using ZED SDK"""
        try:
            self.zed = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = self.image_resolution
            init_params.camera_fps = 30
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            
            if svo_filename:
                init_params.set_from_svo_file(svo_filename)
                self.get_logger().info(f"Loading SVO file: {svo_filename}")
            
            status = self.zed.open(init_params)
            if status != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {status}")
                raise RuntimeError(f"Failed to open ZED camera: {status}")
            
            self.get_logger().info("ZED camera initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Error initializing ZED camera: {e}")
            raise
    
    def init_object_detection(self):
        """Initialize ZED object detection with custom detector mode"""
        try:
            # Enable positional tracking FIRST if tracking is enabled
            # Positional tracking must be enabled before object detection when tracking is enabled
            if self.enable_tracking:
                positional_tracking_parameters = sl.PositionalTrackingParameters()
                zed_error = self.zed.enable_positional_tracking(positional_tracking_parameters)
                if zed_error != sl.ERROR_CODE.SUCCESS:
                    self.get_logger().error(f"Failed to enable positional tracking: {zed_error}")
                    raise RuntimeError(f"Failed to enable positional tracking: {zed_error}")
            
            # Configure object detection parameters
            self.detection_parameters = sl.ObjectDetectionParameters()
            self.detection_parameters.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
            self.detection_parameters.enable_tracking = self.enable_tracking
            self.detection_parameters.enable_segmentation = True  # Enable mask generation
            
            # Enable object detection (after positional tracking if tracking is enabled)
            zed_error = self.zed.enable_object_detection(self.detection_parameters)
            if zed_error != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to enable object detection: {zed_error}")
                raise RuntimeError(f"Failed to enable object detection: {zed_error}")
            
            # Runtime parameters
            self.detection_runtime_parameters = sl.ObjectDetectionRuntimeParameters()
            self.detection_runtime_parameters.detection_confidence_threshold = 30  # 30% minimum confidence
            
            self.get_logger().info("Object detection initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Error initializing object detection: {e}")
            raise
    
    def process_frame(self):
        """Main processing loop"""
        if self.zed is None:
            return
        
        # Grab frame
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return
        
        # Retrieve left image
        self.zed.retrieve_image(self.left_image, sl.VIEW.LEFT)
        
        # Get image as numpy array for detection
        image_np = self.left_image.get_data()
        
        # Run 2D detection
        detections_2d = self.detector.detect(image_np)
        
        # Convert detections to ZED SDK format
        objects_in = []
        for det in detections_2d:
            custom_box = sl.CustomBoxObjectData()
            custom_box.unique_object_id = sl.generate_unique_id()
            custom_box.probability = det['conf']
            custom_box.label = det['class_id']
            
            # Convert bbox to ZED format (4 points as numpy array: top-left, top-right, bottom-right, bottom-left)
            x1, y1, x2, y2 = det['bbox']
            # Format: numpy array of shape (4, 2) where each row is [x, y]
            custom_box.bounding_box_2d = np.array([
                [x1, y1],  # top-left (A)
                [x2, y1],  # top-right (B)
                [x2, y2],  # bottom-right (C)
                [x1, y2]   # bottom-left (D)
            ], dtype=np.float32)
            custom_box.is_grounded = False  # Set to True if objects are on the ground plane
            
            objects_in.append(custom_box)
        
        # Ingest detections into ZED SDK
        if objects_in:
            self.zed.ingest_custom_box_objects(objects_in)
        
        # Retrieve 3D tracked objects
        objects = sl.Objects()
        zed_error = self.zed.retrieve_custom_objects(objects, self.detection_runtime_parameters)
        
        if zed_error == sl.ERROR_CODE.SUCCESS:
            # Publish detections
            self.publish_detections(objects)
            
            # Create annotated image
            annotated_image = self.create_annotated_image(image_np, objects, detections_2d)
            self.publish_annotated_image(annotated_image)
    
    def create_annotated_image(self, image, objects, detections_2d):
        """Create annotated image with bounding boxes and labels"""
        annotated = image.copy()
        
        # Draw 2D bounding boxes from custom detector
        for det in detections_2d:
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"Class {det['class_id']}: {det['conf']:.2f}",
                       (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw 3D tracked objects
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                # Get 2D bounding box
                bbox_2d = obj.bounding_box_2d
                if len(bbox_2d) >= 4:
                    # Convert to integer points
                    pts = np.array([[int(p[0]), int(p[1])] for p in bbox_2d], np.int32)
                    cv2.polylines(annotated, [pts], True, (255, 0, 0), 2)
                    
                    # Draw ID and position
                    center = np.mean(pts, axis=0).astype(int)
                    label = f"ID:{obj.id} ({obj.position[0]:.2f}, {obj.position[1]:.2f}, {obj.position[2]:.2f})"
                    cv2.putText(annotated, label, (center[0], center[1] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        
        return annotated
    
    def publish_annotated_image(self, image):
        """Publish annotated image"""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'zed_camera_center'
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing annotated image: {e}")
    
    def publish_detections(self, objects):
        """Publish 3D object detections as ROS2 MarkerArray"""
        marker_array = MarkerArray()
        
        for idx, obj in enumerate(objects.object_list):
            if obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK:
                continue
            
            # Create marker for 3D position
            marker = Marker()
            marker.header.frame_id = 'zed_camera_center'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = obj.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = float(obj.position[0])
            marker.pose.position.y = float(obj.position[1])
            marker.pose.position.z = float(obj.position[2])
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Set color based on label
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
            # Add text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.id = obj.id + 10000
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
            text_marker.text = f"ID:{obj.id} Label:{obj.raw_label} Pos:({obj.position[0]:.2f},{obj.position[1]:.2f},{obj.position[2]:.2f})"
            
            marker_array.markers.append(text_marker)
            
            # Draw 3D bounding box if available
            if len(obj.bounding_box_3d) >= 8:
                bbox_marker = Marker()
                bbox_marker.header = marker.header
                bbox_marker.id = obj.id + 20000
                bbox_marker.type = Marker.LINE_LIST
                bbox_marker.action = Marker.ADD
                bbox_marker.scale.x = 0.02
                bbox_marker.color.a = 1.0
                bbox_marker.color.r = 0.0
                bbox_marker.color.g = 1.0
                bbox_marker.color.b = 0.0
                
                # Define edges of 3D bounding box (12 edges)
                bbox_3d = obj.bounding_box_3d
                edges = [
                    [0, 1], [1, 2], [2, 3], [3, 0],  # front face
                    [4, 5], [5, 6], [6, 7], [7, 4],  # back face
                    [0, 4], [1, 5], [2, 6], [3, 7]   # connecting edges
                ]
                
                for edge in edges:
                    if edge[0] < len(bbox_3d) and edge[1] < len(bbox_3d):
                        p1 = bbox_3d[edge[0]]
                        p2 = bbox_3d[edge[1]]
                        bbox_marker.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
                        bbox_marker.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
                
                marker_array.markers.append(bbox_marker)
        
        self.detections_pub.publish(marker_array)
    
    def destroy_node(self):
        """Cleanup on node destruction"""
        if self.zed is not None:
            self.zed.disable_object_detection()
            if self.enable_tracking:
                self.zed.disable_positional_tracking()
            self.zed.close()
        super().destroy_node()
