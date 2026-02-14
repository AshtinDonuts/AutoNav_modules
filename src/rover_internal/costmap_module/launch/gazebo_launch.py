#!/usr/bin/env python3
"""
Gazebo Launch Script
Launches Gazebo with simple corridor world and spawns robot
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Gazebo simulation."""
    
    # Get package directories
    nav_sandbox_dir = get_package_share_directory('autonomous_costmap')
    worlds_dir = os.path.join(nav_sandbox_dir, 'worlds')
    world_file = os.path.join(worlds_dir, 'simple_corridor.world')
    urdf_dir = os.path.join(nav_sandbox_dir, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'rover.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Gazebo server (gzserver) - runs physics simulation with ROS init + factory
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '-s', 'libgazebo_ros_init.so',      # provides /unpause_physics service
            '-s', 'libgazebo_ros_factory.so',   # allows spawn_entity
        ],
        output='screen'
    )
    
    # Gazebo client (gzclient) - GUI
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn robot in Gazebo at origin using gazebo_ros plugin
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'rover',
            '-file', urdf_file,
            '-x', '0',
            '-y', '0',
            '-z', '0.1',  # base center at 0.1 -> wheels radius 0.1 touch ground
            '-unpause'
        ],
        output='screen'
    )
    
    # Robot State Publisher - publishes robot_description topic for Gazebo
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,  # Use Gazebo time
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': True,
        }]
    )
    
    # Create launch description
    ld = LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
    
    return ld
