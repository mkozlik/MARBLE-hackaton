from setuptools import setup
import os
from glob import glob
package_name = 'grpc_ros_adapter'
submodules = [f'{package_name}/protobuf', f'{package_name}/services', f'{package_name}/grpc_utils']
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, *submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labust',
    maintainer_email='labust@fer.hr',
    description='Bridges and translates messages between ROS and Unity using gRPC',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'server = {package_name}.server:main',
            f'volume_disp_publisher = {package_name}.volume_disp_publisher:main',
            f'GNSSSubscriber = {package_name}.GNSSSubscriber:main',
            f'larvaeCountNode = {package_name}.larvaeCountNode:main'
        ],
    },
)

