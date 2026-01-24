#!/usr/bin/env python3
"""
ZED VSLAM Node2 Script

Entry point script for the ZED VSLAM node2 with rtabmap_ros integration.
This script handles command-line arguments and launches the ROS2 node.
"""

import sys
import argparse
import rclpy
from rclpy.utilities import remove_ros_args

from mapping_module.nodes.zed_vslam_node2 import ZEDVSLAMNode


def main(args=None):
    """Main function"""
    parser = argparse.ArgumentParser(description='ZED VSLAM Node2 with RTABMap Integration')
    parser.add_argument('--svo-filename', type=str, default=None, nargs='?', const=None,
                       help='Path to SVO file to replay (optional)')
    parser.add_argument('--area-file', type=str, default=None, nargs='?', const=None,
                       help='Path to area memory file (.area) for loading/saving')
    parser.add_argument('--publish-rate', type=float, default=10.0,
                       help='Publishing rate in Hz (default: 10.0)')
    
    # Parse ROS2 args first to separate ROS2 args from node args
    rclpy.init(args=args)
    
    # Get node args (remove ROS2-specific arguments)
    if args:
        node_args = remove_ros_args(args)
    else:
        node_args = sys.argv[1:]
    
    # Filter out empty string arguments and any remaining ROS2 arguments
    # Also remove flag-value pairs where the value is empty
    filtered_args = []
    i = 0
    while i < len(node_args):
        arg = node_args[i]
        
        # Skip ROS2-specific arguments
        if arg in ['--ros-args', '-r'] or arg.startswith('__node:=') or arg.startswith('--params-file'):
            i += 1
            continue
        
        # Skip empty strings
        if arg == '':
            i += 1
            continue
        
        # Check if this is a flag followed by an empty string value
        if arg in ['--svo-filename', '--area-file']:
            if i + 1 < len(node_args) and node_args[i + 1] == '':
                # Skip both the flag and the empty value
                i += 2
                continue
        
        filtered_args.append(arg)
        i += 1
    
    # Use parse_known_args to ignore any remaining unknown arguments (like ROS2 args)
    parsed_args, unknown = parser.parse_known_args(filtered_args)
    
    # Handle empty strings as None
    svo_filename = parsed_args.svo_filename if parsed_args.svo_filename and parsed_args.svo_filename != '' else None
    area_file = parsed_args.area_file if parsed_args.area_file and parsed_args.area_file != '' else None
    
    # Create node
    node = None
    try:
        node = ZEDVSLAMNode(
            svo_filename=svo_filename,
            area_file_path=area_file,
            publish_rate=parsed_args.publish_rate
        )
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down...')
    except Exception as e:
        import traceback
        print(f"Error: {e}")
        print(f"Traceback: {traceback.format_exc()}")
        print("\nNOTE: If you see 'AttributeError: _ARRAY_API not found' or NumPy errors,")
        print("this is due to NumPy version incompatibility with ZED SDK.")
        print("Solution: Install NumPy < 2.0: pip install 'numpy<2.0'")
        if node:
            node.get_logger().error(f"Fatal error: {e}")
            node.get_logger().error(f"Traceback: {traceback.format_exc()}")
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
