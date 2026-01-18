#!/usr/bin/env python3
"""
3D Object Detection Script

Entry point script for the ZED 3D Object Detection node.
This script handles command-line arguments and launches the ROS2 node.
"""

import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args

from mapping_module.nodes.zed_3d_detector_node import ZED3DDetector


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
    
    # Get node args (remove ROS2-specific arguments)
    node_args = remove_ros_args(args) if args else sys.argv[1:]
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
