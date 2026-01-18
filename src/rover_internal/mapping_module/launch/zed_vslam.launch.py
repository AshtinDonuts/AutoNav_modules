"""
Launch file for ZED VSLAM with occupancy tracking
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    svo_file_arg = DeclareLaunchArgument(
        'svo_file',
        default_value='',
        description='Path to SVO file for replay (optional)'
    )
    
    area_file_arg = DeclareLaunchArgument(
        'area_file',
        default_value='',
        description='Path to area memory file (.area) for loading/saving'
    )
    
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.05',
        description='Occupancy grid resolution in meters'
    )
    
    grid_width_arg = DeclareLaunchArgument(
        'grid_width',
        default_value='100',
        description='Occupancy grid width in cells'
    )
    
    grid_height_arg = DeclareLaunchArgument(
        'grid_height',
        default_value='100',
        description='Occupancy grid height in cells'
    )
    
    min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='-0.5',
        description='Minimum height filter in meters'
    )
    
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='2.0',
        description='Maximum height filter in meters'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    # Build arguments list conditionally
    node_args = [
        '--grid-resolution', LaunchConfiguration('grid_resolution'),
        '--grid-width', LaunchConfiguration('grid_width'),
        '--grid-height', LaunchConfiguration('grid_height'),
        '--min-height', LaunchConfiguration('min_height'),
        '--max-height', LaunchConfiguration('max_height'),
        '--publish-rate', LaunchConfiguration('publish_rate'),
    ]
    
    # Add optional arguments only if provided
    svo_file_config = LaunchConfiguration('svo_file')
    area_file_config = LaunchConfiguration('area_file')
    
    # ZED VSLAM node
    zed_vslam_node = Node(
        package='mapping_module',
        executable='zed_vslam',
        name='zed_vslam_node',
        output='screen',
        arguments=node_args + [
            ['--svo-filename', svo_file_config],
            ['--area-file', area_file_config],
        ]
    )
    
    return LaunchDescription([
        svo_file_arg,
        area_file_arg,
        grid_resolution_arg,
        grid_width_arg,
        grid_height_arg,
        min_height_arg,
        max_height_arg,
        publish_rate_arg,
        zed_vslam_node,
        LogInfo(msg='ZED VSLAM node launched with occupancy tracking')
    ])
