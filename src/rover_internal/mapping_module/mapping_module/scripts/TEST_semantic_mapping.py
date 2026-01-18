#!/usr/bin/env python3
"""
ZED Semantic Mapping Script

Entry point script for the ZED VSLAM node with semantic segmentation.
This script handles command-line arguments and launches the ROS2 node.
"""

import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args

from mapping_module.nodes.TEST_semantic_map_node import ZEDVSLAMNode


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='ZED VSLAM Node with Semantic Mapping')
    parser.add_argument('--svo-filename', type=str, default=None,
                       help='Path to SVO file to replay (optional)')
    parser.add_argument('--area-file', type=str, default=None,
                       help='Path to area memory file (.area) for loading/saving')
    parser.add_argument('--grid-resolution', type=float, default=0.05,
                       help='Occupancy grid resolution in meters (default: 0.05)')
    parser.add_argument('--grid-width', type=int, default=100,
                       help='Occupancy grid width in cells (default: 100)')
    parser.add_argument('--grid-height', type=int, default=100,
                       help='Occupancy grid height in cells (default: 100)')
    parser.add_argument('--min-height', type=float, default=-0.5,
                       help='Minimum height filter in meters (default: -0.5)')
    parser.add_argument('--max-height', type=float, default=2.0,
                       help='Maximum height filter in meters (default: 2.0)')
    parser.add_argument('--publish-rate', type=float, default=10.0,
                       help='Publishing rate in Hz (default: 10.0)')
    parser.add_argument('--maskrcnn-config', type=str, default=None,
                       help='Path to Mask R-CNN config file for semantic segmentation')
    parser.add_argument('--min-image-size', type=int, default=256,
                       help='Minimum image size for Mask R-CNN inference (default: 256)')
    parser.add_argument('--device', type=str, default='cuda',
                       choices=['cuda', 'cpu'],
                       help='Device to run Mask R-CNN inference on (default: cuda)')
    
    # Parse ROS2 args
    rclpy.init(args=args)
    
    # Get node args (remove ROS2-specific arguments)
    node_args = remove_ros_args(args) if args else sys.argv[1:]
    parsed_args = parser.parse_args(node_args)
    
    # Handle empty strings as None
    svo_filename = parsed_args.svo_filename if parsed_args.svo_filename and parsed_args.svo_filename != '' else None
    area_file = parsed_args.area_file if parsed_args.area_file and parsed_args.area_file != '' else None
    maskrcnn_config = parsed_args.maskrcnn_config if parsed_args.maskrcnn_config and parsed_args.maskrcnn_config != '' else None
    
    # Create node
    try:
        node = ZEDVSLAMNode(
            svo_filename=svo_filename,
            area_file_path=area_file,
            grid_resolution=parsed_args.grid_resolution,
            grid_width=parsed_args.grid_width,
            grid_height=parsed_args.grid_height,
            min_height=parsed_args.min_height,
            max_height=parsed_args.max_height,
            publish_rate=parsed_args.publish_rate,
            maskrcnn_config=maskrcnn_config,
            min_image_size=parsed_args.min_image_size,
            device=parsed_args.device
        )
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
