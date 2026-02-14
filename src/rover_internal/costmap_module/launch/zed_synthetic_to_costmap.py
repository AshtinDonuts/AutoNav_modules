#!/usr/bin/env python3
"""
ZED Synthetic PointCloud to Costmap Launch Script
For testing synthetic pointcloud → costmap pipeline
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for synthetic ZED pointcloud to costmap."""
    
    # Get package directories
    pkg_dir = get_package_share_directory('autonomous_costmap')
    config_dir = os.path.join(pkg_dir, 'config')
    maps_dir = os.path.join(pkg_dir, 'maps')
    urdf_dir = os.path.join(pkg_dir, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'rover.urdf')
    
    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'empty_map.yaml'),
        description='Full path to map file to load'
    )
    
    # Get launch configurations
    map_yaml = LaunchConfiguration('map')
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
            'publish_timer_hz': 10.0,
        }]
    )

    # Lifecycle Manager - manages map_server lifecycle
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['map_server']
        }]
    )
    
    # Synthetic PointCloud Node - Simulates ZED pointcloud output
    install_prefix = str(Path(pkg_dir).parent.parent)
    synthetic_pointcloud_exe = os.path.join(install_prefix, 'bin', 'synthetic_pointcloud_node')
    
    synthetic_pointcloud_node = ExecuteProcess(
        cmd=[
            synthetic_pointcloud_exe,
            '--ros-args',
            '-p', 'topic:=/synthetic/points',
            '-p', 'pattern:=obstacles',
            '-p', 'rate_hz:=10.0',
            '-p', 'frame_id:=zed_left_camera_frame',
        ],
        output='screen'
    )
    
    # Costmap Node - Subscribes to synthetic/points for testing
    nav2_params_file = os.path.join(config_dir, 'synthetic_costmap.yaml')
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='costmap',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Delayed costmap lifecycle commands
    costmap_configure_cmd = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'configure'],
                output='screen',
                shell=False
            )
        ]
    )
    
    costmap_activate_cmd = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'activate'],
                output='screen',
                shell=False
            )
        ]
    )
    
    # Get executable paths
    waypoint_marker_exe = os.path.join(install_prefix, 'bin', 'waypoint_marker_node')
    costmap_visualizer_exe = os.path.join(install_prefix, 'bin', 'costmap_visualizer')
    simulated_base_exe = os.path.join(install_prefix, 'bin', 'simulated_base')
    
    waypoint_marker_node = ExecuteProcess(
        cmd=[waypoint_marker_exe],
        output='screen'
    )
    
    costmap_visualizer_node = ExecuteProcess(
        cmd=[costmap_visualizer_exe],
        output='screen'
    )
    
    simulated_base_node = ExecuteProcess(
        cmd=[simulated_base_exe],
        output='screen'
    )
    
    # Robot State Publisher
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': False,
        }]
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': urdf_content,
    #         'use_gui': False,
    #     }]
    # )
    
    # # Static TF: map -> odom
    # map_to_odom_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )
    
    # Create launch description
    ld = LaunchDescription([
        map_yaml_arg,
        map_server_node,
        lifecycle_manager_node,
        synthetic_pointcloud_node,      # Synthetic pointcloud → /synthetic/points
        costmap_configure_cmd,
        costmap_activate_cmd,
        costmap_node,                   # Costmap subscribes to /synthetic/points
        robot_state_publisher,
        # map_to_odom_tf,
        # joint_state_publisher,
        simulated_base_node,
        waypoint_marker_node,
        costmap_visualizer_node,
    ])
    
    return ld
