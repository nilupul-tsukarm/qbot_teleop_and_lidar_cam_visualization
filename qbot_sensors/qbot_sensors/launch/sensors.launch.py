"""Start the QBot Platform lidar + RealSense nodes and basic static transforms.

  ros2 launch qbot_sensors sensors.launch.py
  ros2 launch qbot_sensors sensors.launch.py camera:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    camera = LaunchConfiguration('camera')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),

        Node(package='qbot_sensors', executable='lidar_node', name='qbot_lidar',
             output='screen', condition=IfCondition(lidar),
             parameters=[{'frame_id': 'lidar',
                          'reverse_direction': True,
                          'angle_offset_deg': 0.0}]),

        Node(package='qbot_sensors', executable='realsense_node', name='qbot_realsense',
             output='screen', condition=IfCondition(camera),
             parameters=[{'width': 640, 'height': 480, 'fps': 30.0}]),

        # base_link -> lidar  (x y z yaw pitch roll). Measure your robot and edit.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.0', '--y', '0.0', '--z', '0.20',
                        '--frame-id', 'base_link', '--child-frame-id', 'lidar']),

        # base_link -> camera optical frame (optical: z forward, x right, y down)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0.10', '--y', '0.0', '--z', '0.15',
                        '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                        '--frame-id', 'base_link',
                        '--child-frame-id', 'camera_color_optical_frame']),
    ])
