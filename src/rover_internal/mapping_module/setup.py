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
            'launch/zed_3dpc.launch.py',
            'launch/new_rtab.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/zed_vslam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khw',
    maintainer_email='kingwonghku@gmail.com',
    description='Mapping module for rover with ZED camera VSLAM and RTAB-Map integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_3dpc = mapping_module.scripts.zed_3dpc:main',
        ],
    },
)
