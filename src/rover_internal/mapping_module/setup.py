from setuptools import find_packages, setup

package_name = 'mapping_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/zed_vslam.launch.py', 
            'launch/zed_3dpc.launch.py',
            'launch/zed_vslam_rtabmap.launch.py',
            'launch/new_rtab.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/zed_vslam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khw',
    maintainer_email='kingwonghku@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_3d_detector = mapping_module.scripts.3d_detector:main',
            'zed_2d_detector = mapping_module.scripts.2d_detector:main',
            'zed_vslam = mapping_module.scripts.zed_vslam:main',
            'zed_vslam2-node = mapping_module.scripts.zed_vslam2:main',
            'zed_3dpc = mapping_module.scripts.zed_3dpc:main',
        ],
    },
)
