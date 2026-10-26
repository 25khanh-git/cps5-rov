"""Chạy trên laptop điều khiển khi lái robot thật: tay cầm (joy_node) + teleop_node.

  ros2 launch rov_bringup topside.launch.py
Không có tay cầm: topside.launch.py joy:=false rồi ở terminal khác: ros2 run rov_teleop keyboard_teleop
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rov_bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('joy', default_value='true', description='Chạy joy_node'),
        Node(package='joy', executable='joy_node', output='screen',
             parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
             condition=IfCondition(LaunchConfiguration('joy'))),
        Node(package='rov_teleop', executable='teleop_node', output='screen',
             parameters=[os.path.join(cfg, 'teleop.yaml')]),
    ])
