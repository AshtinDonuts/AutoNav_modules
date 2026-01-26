# Requirements:
#   A ZED2i camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
#
# Example:
#
#   SLAM with live camera (saves map):
#     $ ros2 launch mapping_module new_rtab.launch.py rviz:=true rtabmap_viz:=true
#
#   SLAM with fresh start (delete previous map):
#     $ ros2 launch mapping_module new_rtab.launch.py delete_db:=true
#
#   SLAM with rosbag (set use_sim_time:=true):
#     $ ros2 launch mapping_module new_rtab.launch.py use_sim_time:=true rviz:=true rtabmap_viz:=true
#     $ ros2 bag play <your_bag> --clock
#
#   Localization mode (requires existing map):
#     $ ros2 launch mapping_module new_rtab.launch.py localization:=true rviz:=true
#
#   View saved map:
#     $ rtabmap-databaseViewer ~/.ros/rtabmap.db
#
#   Export map to other formats:
#     $ ros2 run rtabmap_slam rtabmap-export --db ~/.ros/rtabmap.db --output my_map.pcd

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context: LaunchContext, *args, **kwargs):
    
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_zed_odometry = LaunchConfiguration('use_zed_odometry')
    subscribe_scan = LaunchConfiguration('subscribe_scan')
    scan_topic = LaunchConfiguration('scan_topic')
    delete_db = LaunchConfiguration('delete_db')
    
    subscribe_scan_val = subscribe_scan.perform(context)
    use_zed_odometry_val = use_zed_odometry.perform(context)
    delete_db_val = delete_db.perform(context)

    parameters={
          'frame_id':'zed_camera_link',
          'odom_frame_id':'odom',
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_rgbd':True,
          'subscribe_scan':subscribe_scan_val.lower() == 'true',
          'approx_sync':True,  # Approximate sync for live camera (more forgiving with timestamps)
          'sync_queue_size': 10,
          'wait_imu_to_init':True,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          'Reg/Strategy':              '1',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'false',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          'Grid/FromDepth':            'false',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          'RGBD/LocalRadius':          '5',       # limit length of proximity detections
          'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'false',
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.05'
    }
    
    # ZED camera topic remappings
    # Using new ZED wrapper topic format (v5.1.0+)
    remappings=[
         ('rgb/image',       '/zed/zed_node/rgb/color/rect/image'),
         ('depth/image',     '/zed/zed_node/depth/depth_registered'),
         ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
         ('imu',             '/zed/zed_node/imu/data')]
    
    # Add scan topic remapping if subscribing to scan
    if subscribe_scan_val.lower() == 'true':
        remappings.append(('scan', scan_topic.perform(context)))
    
    # Add odometry remapping if using ZED odometry
    if use_zed_odometry_val.lower() == 'true':
        remappings.append(('odom', '/zed/zed_node/odom'))
    else:
        parameters['subscribe_odom_info'] = True
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    nodes = [
        # RGBD sync node
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[parameters,
              {'rgb_image_transport':'raw',  # Use raw for live camera, can be 'compressed' for rosbag
               'depth_image_transport':'raw',  # Use raw for live camera, can be 'compressedDepth' for rosbag
               'approx_sync_max_interval': 0.02}],
            remappings=[('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
                        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
                        ('depth/image', '/zed/zed_node/depth/depth_registered')]
            ),
        
        # Visual odometry (only if not using ZED odometry)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=[parameters],
            remappings=remappings,
        ),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']
        ),
        
        # # SLAM mode:
        # Node(
        #     condition=UnlessCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings,
        #     arguments=['-d'] if delete_db_val.lower() == 'true' else []), # Conditionally delete database
            
        # # Localization mode:
        # Node(
        #     condition=IfCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters,
        #       {'Mem/IncrementalMemory':'False',
        #        'Mem/InitWMWithAllNodes':'True'}],
        #     remappings=remappings),

        # # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings
            ),
        
        # Rviz is better for examining specific topics
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]
            )
        ]
    
    return nodes

def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time (set to true for rosbag playback).'),
        DeclareLaunchArgument('use_zed_odometry', default_value='false', description='Use ZED\'s computed odometry instead of RTAB-Map\'s visual odometry.'),
        DeclareLaunchArgument('subscribe_scan', default_value='false', description='Subscribe to laser scan topic.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Laser scan topic name.'),
        DeclareLaunchArgument('delete_db', default_value='false', description='Delete previous database on startup (set to true for fresh start).'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        OpaqueFunction(function=launch_setup),
    ])
