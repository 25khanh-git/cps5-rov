"""Chạy trên Raspberry Pi 4 trong robot thật: MAVROS (nối Pixhawk 6C) + camera USB + safety_node.

Laptop chạy topside.launch.py (tay cầm + teleop). Hai máy cùng ROS_DOMAIN_ID, nối qua cáp Ethernet.

Ví dụ:
  ros2 launch rov_bringup real.launch.py                                   # USB: /dev/ttyACM0
  ros2 launch rov_bringup real.launch.py fcu_url:=/dev/serial0:921600      # TELEM2 -> UART GPIO
  ros2 launch rov_bringup real.launch.py gcs_url:=udp://@192.168.2.1:14550 # QGroundControl trên laptop
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
    mavros_share = get_package_share_directory('mavros')

    mavros = Node(
        package='mavros', executable='mavros_node', output='screen', respawn=True,
        respawn_delay=3.0,
        parameters=[
            os.path.join(mavros_share, 'launch', 'apm_pluginlists.yaml'),
            os.path.join(mavros_share, 'launch', 'apm_config.yaml'),
            os.path.join(cfg, 'mavros.yaml'),
            {'fcu_url': LaunchConfiguration('fcu_url'),
             'gcs_url': LaunchConfiguration('gcs_url'),
             'tgt_system': 1, 'tgt_component': 1},
        ],
    )
    camera = Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='camera', output='screen',
        parameters=[{'video_device': LaunchConfiguration('video_device'),
                     'image_size': [640, 480]}],
        remappings=[('image_raw', '/camera/image_raw'), ('camera_info', '/camera/camera_info')],
        condition=IfCondition(LaunchConfiguration('camera')),
    )
    safety = Node(package='rov_safety', executable='safety_node', output='screen',
                  parameters=[os.path.join(cfg, 'safety.yaml')])

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200',
                              description='Pixhawk 6C qua USB (hoặc /dev/serial0:921600 qua TELEM2)'),
        DeclareLaunchArgument('gcs_url', default_value='', description='Chuyển tiếp cho QGroundControl'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        mavros, camera, safety,
    ])
