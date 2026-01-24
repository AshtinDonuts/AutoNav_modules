"""
Launch file for ZED 3D Point Cloud publisher

Mainly for testing purpose. Will be integrated into zed_vslam module
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""
    
    # Declare launch arguments
    svo_file_arg = DeclareLaunchArgument(
        'svo_file',
        default_value='',
        description='Path to SVO file for replay (optional)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/zed/points',
        description='Topic name for point cloud publishing'
    )
    
    depth_mode_arg = DeclareLaunchArgument(
        'depth_mode',
        default_value='NEURAL',
        description='Depth mode: NEURAL (requires TensorRT), PERFORMANCE, QUALITY, or ULTRA'
    )
    
    # Build arguments list
    svo_file_config = LaunchConfiguration('svo_file')
    
    # ZED 3D Point Cloud node
    zed_3dpc_node = Node(
        package='mapping_module',
        executable='zed_3dpc',
        name='zed_3dpc_node',
        output='screen',
        arguments=[
            '--publish-rate', LaunchConfiguration('publish_rate'),
            '--topic-name', LaunchConfiguration('topic_name'),
            '--svo-filename', svo_file_config,
            '--depth-mode', LaunchConfiguration('depth_mode'),
        ]
    )
    
    return LaunchDescription([
        svo_file_arg,
        publish_rate_arg,
        topic_name_arg,
        depth_mode_arg,
        zed_3dpc_node,
        LogInfo(msg='ZED 3D Point Cloud node launched')
    ])
