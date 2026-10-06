from glob import glob
from setuptools import setup

package_name = 'qbot_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nilupul',
    description='ROS 2 publishers for the QBot Platform lidar and RealSense camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_node = qbot_sensors.lidar_node:main',
            'realsense_node = qbot_sensors.realsense_node:main',
        ],
    },
)
