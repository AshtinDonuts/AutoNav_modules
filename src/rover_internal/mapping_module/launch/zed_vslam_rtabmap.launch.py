"""
Launch file for ZED VSLAM with RTABMap integration


This launch file launches both:
- ZED VSLAM node2 (publishes RGB/depth images and odometry)
- RTABMap ROS (generates occupancy grid from sensor data)

Usage:
cd ~/Dev/rover_ROS_ws
colcon build --packages-select mapping_module
source install/setup.bash

# Launch both nodes together
ros2 launch mapping_module zed_vslam_rtabmap.launch.py

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description"""

    # Declare launch arguments for ZED VSLAM node
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
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )
    
    # RTABMap arguments
    rtabmap_args = DeclareLaunchArgument(
        'rtabmap_args',
        default_value='',
        description='Additional arguments to pass to rtabmap node'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RTABMap parameters
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/rgb/image_raw',
        description='RGB image topic for rtabmap'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth/image_raw',
        description='Depth image topic for rtabmap'
    )
    
    rgb_camera_info_topic_arg = DeclareLaunchArgument(
        'rgb_camera_info_topic',
        default_value='/rgb/camera_info',
        description='RGB camera info topic for rtabmap'
    )
    
    depth_camera_info_topic_arg = DeclareLaunchArgument(
        'depth_camera_info_topic',
        default_value='/depth/camera_info',
        description='Depth camera info topic for rtabmap'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic for rtabmap'
    )
    
    # ZED VSLAM Node2
    zed_vslam_node = Node(
        package='mapping_module',
        executable='zed_vslam_node2',
        name='zed_vslam_node',
        output='screen',
        parameters=[{
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'rgb_camera_info_topic': LaunchConfiguration('rgb_camera_info_topic'),
            'depth_camera_info_topic': LaunchConfiguration('depth_camera_info_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'rtabmap_grid_map_topic': '/rtabmap/grid_map',
            'occupancy_grid_topic': '/zed_vslam/occupancy_grid',
            'camera_frame_id': 'zed_camera_center',
        }],
        arguments=[
            '--publish-rate', LaunchConfiguration('publish_rate'),
            '--svo-filename', LaunchConfiguration('svo_file'),
            '--area-file', LaunchConfiguration('area_file'),
        ]
    )
    
    # RTABMap SLAM node
    # Note: rtabmap_ros uses parameter files or launch includes typically
    # This is a basic configuration - you may want to use rtabmap_launch instead
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'zed_camera_center',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'queue_size': 20,  # Increased for better synchronization
            'sync_queue_size': 20,  # Increased for better synchronization
            # Topic names (rtabmap expects these as parameters)
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'rgb_camera_info_topic': LaunchConfiguration('rgb_camera_info_topic'),
            'depth_camera_info_topic': LaunchConfiguration('depth_camera_info_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            # Grid map settings
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/MaxGroundHeight': '-0.5',
            'Grid/RangeMax': '20.0',
            'Grid/RangeMin': '0.1',
            'Grid/3D': 'false',  # 2D occupancy grid
            'Grid/FromDepth': 'true',
            'Grid/CellSize': '0.05',
            # RTABMap settings
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/CreateOccupancyGrid': 'true',
            'RGBD/OccupancyGridMapEmptyRayTracing': 'true',
            'RGBD/OccupancyGridMapFullRayTracing': 'true',
        }],
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_topic')),
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('rgb/camera_info', LaunchConfiguration('rgb_camera_info_topic')),
            ('depth/camera_info', LaunchConfiguration('depth_camera_info_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        svo_file_arg,
        area_file_arg,
        publish_rate_arg,
        rtabmap_args,
        use_sim_time_arg,
        rgb_topic_arg,
        depth_topic_arg,
        rgb_camera_info_topic_arg,
        depth_camera_info_topic_arg,
        odom_topic_arg,
        # Nodes
        zed_vslam_node,
        rtabmap_node,
        LogInfo(msg='ZED VSLAM with RTABMap launched'),
        LogInfo(msg='ZED VSLAM node publishing RGB/depth images and odometry'),
        LogInfo(msg='RTABMap generating occupancy grid from sensor data'),
    ])
