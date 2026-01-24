#!/usr/bin/env python3
"""
ZED 3D Point Cloud Script

Entry point script for the ZED 3D Point Cloud node.
This script handles command-line arguments and launches the ROS2 node.
"""

import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args

from mapping_module.nodes.zed_3dpc_node import ZedPointCloudNode


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='ZED 3D Point Cloud Node')
    parser.add_argument('--svo-filename', type=str, default=None,
                       help='Path to SVO file to replay (optional)')
    parser.add_argument('--publish-rate', type=float, default=10.0,
                       help='Publishing rate in Hz (default: 10.0)')
    parser.add_argument('--topic-name', type=str, default='/zed/points',
                       help='Topic name for point cloud publishing (default: /zed/points)')
    parser.add_argument('--depth-mode', type=str, default='PERFORMANCE',
                       choices=['NEURAL', 'PERFORMANCE', 'QUALITY', 'ULTRA'],
                       help='Depth mode: NEURAL (requires TensorRT), PERFORMANCE, QUALITY, or ULTRA (default: PERFORMANCE)')
    
    # Get node args (remove ROS2-specific arguments like --ros-args, -r, etc.)
    # remove_ros_args removes --ros-args and remapping arguments
    if args is None:
        # When called from command line, use sys.argv
        # remove_ros_args expects full sys.argv and returns args without ROS2 args
        node_args = remove_ros_args(sys.argv)
    else:
        # When called programmatically, remove ROS args from provided args
        node_args = remove_ros_args(args)
    
    # Parse ROS2 args (this handles --ros-args and remapping)
    rclpy.init(args=args)
    
    # Parse our custom arguments (skip script name which is first element)
    parsed_args = parser.parse_args(node_args[1:] if len(node_args) > 0 else [])
    
    # Handle empty strings as None
    svo_filename = parsed_args.svo_filename if parsed_args.svo_filename and parsed_args.svo_filename != '' else None
    
    # Create node
    try:
        node = ZedPointCloudNode(
            svo_filename=svo_filename,
            publish_rate=parsed_args.publish_rate,
            topic_name=parsed_args.topic_name,
            depth_mode=parsed_args.depth_mode
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
