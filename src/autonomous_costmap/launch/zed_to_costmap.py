import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_costmap')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1. Planner Server
        Node(
            package='nav2_planner', 
            executable='planner_server', 
            name='planner_server', 
            output='screen',
            parameters=[params_file]
        ),

        # 2. Your Python Script
        Node(
            package='autonomous_costmap', 
            executable='pcd_to_height_costmap', 
            name='pcd_to_height',
            output='screen'
        ),

        # 3. Nav2 Costmap Node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            output='screen',
            parameters=[params_file]
        ),

        # 4. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server', 'costmap/costmap'],
                'bond_timeout': 10.0 # Double the timeout to 10 seconds
            }]
        ),
    ])