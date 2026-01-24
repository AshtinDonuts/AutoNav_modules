#!/usr/bin/env python3
"""
2D Object Detection Script

Entry point script for the ZED 2D Object Detection node.
This script handles command-line arguments and launches the ROS2 node.

The 2D detector uses a custom bounding box detector (e.g., YOLO) and ingests
the detections into the ZED SDK, which computes 3D positions and tracks objects.

Based on: https://www.stereolabs.com/docs/object-detection/custom-od
"""

import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args

from mapping_module.nodes.zed_2d_detector_node import ZED2DDetector


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='ZED 2D Object Detection Node with Custom Detector')
    parser.add_argument('--model-path', type=str, default=None,
                       help='Path to YOLO model file (e.g., yolov8n.pt). If not provided, uses default YOLOv8n.')
    parser.add_argument('--confidence-threshold', type=float, default=0.5,
                       help='Confidence threshold for detections (default: 0.5)')
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'],
                       help='Device to run inference on (default: cuda)')
    parser.add_argument('--disable-tracking', action='store_true',
                       help='Disable object tracking (default: tracking enabled)')
    parser.add_argument('--svo-filename', type=str, default=None,
                       help='Path to SVO file to replay instead of live camera')
    parser.add_argument('--resolution', type=str, default='HD720',
                       choices=['HD2K', 'HD1080', 'HD720', 'VGA'],
                       help='Camera resolution (default: HD720)')
    
    # Parse ROS2 args
    rclpy.init(args=args)
    
    # Get node args (remove ROS2-specific arguments)
    node_args = remove_ros_args(args) if args else sys.argv[1:]
    parsed_args = parser.parse_args(node_args)
    
    # Convert resolution string to ZED SDK enum
    try:
        import pyzed.sl as sl
        resolution_map = {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA
        }
        image_resolution = resolution_map[parsed_args.resolution]
    except ImportError:
        print("Warning: ZED SDK not available. Resolution setting will be ignored.")
        image_resolution = None
    
    # Handle empty strings as None
    model_path = parsed_args.model_path if parsed_args.model_path and parsed_args.model_path != '' else None
    svo_filename = parsed_args.svo_filename if parsed_args.svo_filename and parsed_args.svo_filename != '' else None
    
    # Create node
    try:
        node = ZED2DDetector(
            model_path=model_path,
            confidence_threshold=parsed_args.confidence_threshold,
            device=parsed_args.device,
            enable_tracking=not parsed_args.disable_tracking,
            svo_filename=svo_filename,
            image_resolution=image_resolution
        )
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
