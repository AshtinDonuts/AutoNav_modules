#!/usr/bin/env python3
"""
Sandbox Launch Script
Launches all necessary components for the navigation sandbox:
- Map Server
- Waypoint Marker Node
- RViz2
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Navigation Sandbox."""
    
    # Get package directories
    nav_sandbox_dir = get_package_share_directory('autonomous_costmap')
    config_dir = os.path.join(nav_sandbox_dir, 'config')
    maps_dir = os.path.join(nav_sandbox_dir, 'maps')
    urdf_dir = os.path.join(nav_sandbox_dir, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'rover.urdf')
    
    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(maps_dir, 'empty_map.yaml'),
        description='Full path to map file to load'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(config_dir, 'rviz_config.rviz'),
        description='Full path to the RViz config file to use'
    )
    
    # Get launch configurations
    map_yaml = LaunchConfiguration('map')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Map Server Node - loads the static map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
            'publish_timer_hz': 10.0,  # Publish at 10 Hz for RViz2 reconnections
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
            'node_names': ['map_server']
        }]
    )
    
    # Delayed commands to configure and activate costmap (wait 3 seconds for node to start)
    costmap_configure_cmd = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'configure'],
                output='screen',
                shell=False
            )
        ]
    )
    
    costmap_activate_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'activate'],
                output='screen',
                shell=False
            )
        ]
    )    
    # Get executable paths
    # nav_sandbox_dir = .../install/nav_sandbox/share/nav_sandbox
    # We need .../install/nav_sandbox/bin/...
    install_prefix = str(Path(nav_sandbox_dir).parent.parent)
    waypoint_marker_exe = os.path.join(install_prefix, 'bin', 'waypoint_marker_node')
    costmap_visualizer_exe = os.path.join(install_prefix, 'bin', 'costmap_visualizer')
    simulated_base_exe = os.path.join(install_prefix, 'bin', 'simulated_base')
    synthetic_pointcloud_exe = os.path.join(install_prefix, 'bin', 'synthetic_pointcloud_node')
    
    # Waypoint Marker Node - publishes goal points
    waypoint_marker_node = ExecuteProcess(
        cmd=[waypoint_marker_exe],
        output='screen'
    )
    
    # Costmap Visualizer Node - monitors data flow
    costmap_visualizer_node = ExecuteProcess(
        cmd=[costmap_visualizer_exe],
        output='screen'
    )

    # Simulated Base - integrates /cmd_vel into odom->base_link for RViz movement
    simulated_base_node = ExecuteProcess(
        cmd=[simulated_base_exe],
        output='screen'
    )

    # Synthetic PointCloud - publishes sensor_msgs/PointCloud2 for voxel layer
    synthetic_pointcloud_node = ExecuteProcess(
        cmd=[synthetic_pointcloud_exe],
        output='screen'
    )
    
    # Local Costmap Node - subscribes to pointcloud and creates obstacle map
    nav2_params_file = os.path.join(config_dir, 'nav2_params.yaml')
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        namespace='costmap',
        output='screen',
        parameters=[nav2_params_file]
    )
        
    # Robot State Publisher - publishes TF transforms for robot model
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

    # Joint State Publisher - publishes joint positions for non-fixed joints
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_gui': False,
        }]
    )
    
    # Static TF: map -> odom (identity transform, robot starts at map origin)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Create launch description
    ld = LaunchDescription([
        map_yaml_arg,                    # Launch argument: map file path
        rviz_config_arg,                 # Launch argument: RViz config file path
        map_server_node,                 # Publishes static map to /map topic
        lifecycle_manager_node,          # Auto-activates map_server
        costmap_configure_cmd,           # Configure costmap after 3 seconds
        costmap_activate_cmd,            # Activate costmap after 4 seconds
        local_costmap_node,              # Creates obstacle costmap from pointcloud
        map_to_odom_tf,                  # Static TF: map → odom (robot start position)
        joint_state_publisher,           # Publishes wheel joint states to /joint_states
        robot_state_publisher,           # Broadcasts TF: base_link → wheels/camera, publishes robot model
        simulated_base_node,             # Integrates /cmd_vel into /odom, broadcasts TF: odom → base_link
        synthetic_pointcloud_node,       # Simulates ZED 2i sensor, publishes PointCloud2 with fan-sweep pattern
        waypoint_marker_node,            # Publishes 5 red sphere markers as navigation goals
        costmap_visualizer_node          # Monitors data flow for debugging
    ])
    
    return ld

