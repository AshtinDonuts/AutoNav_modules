# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   ros2 launch autonomous_camera camera.launch.py camera_model:=zed2i

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Override ZED wrapper parameters without modifying config files directly.
    # NOTE: If upgrading ZED SDK to v5.1+, update sync_remappings topic names below.
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write(
            "---\n"
            "/**:\n"
            "    ros__parameters:\n"
            "        general:\n"
            "            grab_resolution: 'VGA'\n"
            "        depth:\n"
            "            depth_mode: 'NEURAL'\n"
        )

    # Parameters shared across rtabmap_slam, rtabmap_odom, and rtabmap_viz nodes.
    # wait_imu_to_init: RTABMap waits for IMU TF before starting SLAM.
    #   Requires publish_imu_tf: true in ZED override above.
    #   Safe to set because ZED 2i has an IMU and we publish its TF.
    # Note: subscribe_rgbd and wait_imu_to_init are NOT valid for rgbd_sync.
    rtabmap_parameters = [
    {
        'frame_id': 'zed_camera_link',
        'subscribe_rgbd': True,
        'approx_sync': False,
        'wait_imu_to_init': True,
        'Imu/GyroNoise':   '0.00386',
        'Imu/AccNoise':    '0.00699',
        'Imu/GyroWalk':    '0.000438',
        'Imu/AccWalk':     '0.000186',
    }
]

    # Parameters for rgbd_sync only (subset — no rtabmap-specific params)
    sync_parameters = [
        {
            'approx_sync': False,
        }
    ]

    # IMU remapping: RTABMap uses raw IMU alongside ZED odometry for better
    # loop closure and drift correction, even when use_zed_odometry=true.
    rtabmap_remappings = [('imu', '/zed/zed_node/imu/data')]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ['True', 'true']:
        rtabmap_remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        rtabmap_parameters.append({'subscribe_odom_info': True})

    # ZED SDK v5.0.0 topic names.
    # NOTE: v5.1.0 renamed these — update if upgrading:
    #   rgb/image_rect_color      -> rgb/color/rect/image
    #   rgb/camera_info           -> rgb/color/rect/camera_info
    sync_remappings = [
        ('rgb/image',       '/zed/zed_node/rgb/image_rect_color'),
        ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
        ('depth/image',     '/zed/zed_node/depth/depth_registered'),
    ]

    return [
        # Launch ZED camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('zed_wrapper'), 'launch'),
                '/zed_camera.launch.py'
            ]),
            launch_arguments={
                'camera_model':             LaunchConfiguration('camera_model'),
                'ros_params_override_path': zed_override_file.name,
                'publish_tf':               'true',
                'publish_map_tf':           'true',
                'publish_imu_tf':           'true',
            }.items(),
        ),

        # Sync rgb/depth/camera_info into a single rgbd message
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=sync_parameters,
            remappings=sync_remappings,
        ),

        # Visual odometry (only when NOT using ZED's built-in odometry)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
        ),

        # VSLAM — deletes existing DB on start (-d flag)
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            arguments=['-d'],
        ),

        # RTABMap visualizer
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
        ),

        # Uncomment to use RViz instead of rtabmap_viz:
        # Node(
        #     package='rviz2', executable='rviz2', name='rviz2', output='screen',
        #     condition=IfCondition(LaunchConfiguration('rviz')),
        #     arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='true',
            description="Use ZED's built-in odometry instead of RTABMap's rgbd_odometry node."),

        DeclareLaunchArgument(
            'camera_model', default_value='',
            description=(
                "[REQUIRED] The model of the camera. Using a wrong camera model can disable "
                "camera features. Valid choices are: "
                "['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"
            )),

        OpaqueFunction(function=launch_setup),
    ])