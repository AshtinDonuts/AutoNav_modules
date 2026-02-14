from setuptools import find_packages, setup

package_name = 'autonomous_costmap'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/zed_to_costmap.py',
            'launch/zed_synthetic_to_costmap.py',
            'launch/stereo_camera_to_costmap.py',
            'launch/gazebo_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/stereo2costmap.yaml',
            'config/synthetic_costmap.yaml',
            'config/rviz_config.rviz',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/empty_map.yaml',
            'maps/empty_map.pgm',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/rover.urdf',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/simple_corridor.world',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Autonomous Team',
    author_email='user@adelaide.edu.au',
    maintainer='Autonomous Team',
    maintainer_email='user@adelaide.edu.au',
    description='Navigation Sandbox MVP for autonomous navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_marker_node = scripts.waypoint_marker_node:main',
            'costmap_visualizer = scripts.costmap_visualizer:main',
            'simulated_base = scripts.simulated_base:main',
            'synthetic_pointcloud_node = scripts.synthetic_pointcloud_node:main',
        ],
    },
)
