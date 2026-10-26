"""Chạy toàn bộ sim trên laptop (WSL2):

  Gazebo (bể nước + CPS 5)  <-JSON/UDP 9002->  ArduSub SITL  <-MAVLink/UDP 14550->  MAVROS
  + ros_gz_bridge (camera, ground truth) + safety_node + teleop_node (+ joy_node tuỳ chọn)

Ví dụ:
  ros2 launch rov_bringup sim.launch.py                       # có cửa sổ Gazebo
  ros2 launch rov_bringup sim.launch.py gui:=false            # không cửa sổ (nhẹ hơn)
  ros2 launch rov_bringup sim.launch.py render_engine:=ogre   # nếu ogre2 lỗi trên WSL
  ros2 launch rov_bringup sim.launch.py joy:=true             # dùng tay cầm thật
  ros2 launch rov_bringup sim.launch.py chase:=false          # tắt camera bám đuôi
Điều khiển bằng bàn phím (terminal khác):  ros2 run rov_teleop keyboard_teleop
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('rov_bringup')
    mavros_share = get_package_share_directory('mavros')
    cfg = os.path.join(bringup, 'config')

    gui = LaunchConfiguration('gui')
    teleop = LaunchConfiguration('teleop')
    joy = LaunchConfiguration('joy')
    safety = LaunchConfiguration('safety')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('cps5_gazebo'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'gui': gui,
            'camera': LaunchConfiguration('camera'),
            'render_engine': LaunchConfiguration('render_engine'),
            'vehicle': LaunchConfiguration('vehicle'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    # ArduSub SITL khởi động sau Gazebo vài giây (lock-step: SITL chờ dữ liệu từ plugin)
    sitl = TimerAction(period=LaunchConfiguration('sitl_delay'), actions=[ExecuteProcess(
        cmd=['bash', os.path.join(bringup, 'scripts', 'run_sitl.sh'),
             LaunchConfiguration('parm'), LaunchConfiguration('home')],
        output='screen', name='ardusub_sitl')])

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

    safety_node = Node(package='rov_safety', executable='safety_node', output='screen',
                       parameters=[os.path.join(cfg, 'safety.yaml')],
                       condition=IfCondition(safety))
    teleop_node = Node(package='rov_teleop', executable='teleop_node', output='screen',
                       parameters=[os.path.join(cfg, 'teleop.yaml')],
                       condition=IfCondition(teleop))
    # Camera cửa sổ Gazebo bám đuôi robot (mượt, không lắc) — chỉ khi có cửa sổ Gazebo
    chase_camera = Node(package='rov_bringup', executable='chase_camera', output='screen',
                        parameters=[{'distance': LaunchConfiguration('chase_distance'),
                                     'height': LaunchConfiguration('chase_height')}],
                        condition=IfCondition(PythonExpression(
                            ["'", gui, "' == 'true' and '", LaunchConfiguration('chase'), "' == 'true'"])))
    joy_node = Node(package='joy', executable='joy_node', output='screen',
                    parameters=[{'deadzone': 0.05, 'autorepeat_rate': 20.0}],
                    condition=IfCondition(joy))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Mở cửa sổ Gazebo'),
        DeclareLaunchArgument('camera', default_value='true', description='Bật camera mô phỏng'),
        DeclareLaunchArgument('render_engine', default_value='ogre2', description='ogre2 | ogre'),
        DeclareLaunchArgument('vehicle', default_value='', description='YAML thông số robot (trống = mặc định)'),
        DeclareLaunchArgument('world', default_value='', description='YAML thế giới (trống = mặc định)'),
        DeclareLaunchArgument('parm', default_value=os.path.join(cfg, 'ardusub', 'cps5.parm'),
                              description='File tham số ArduSub'),
        DeclareLaunchArgument('home', default_value='21.0047,105.8434,0,0',
                              description='Vị trí home SITL: lat,lon,alt,heading'),
        DeclareLaunchArgument('sitl_delay', default_value='4.0', description='Giây chờ trước khi chạy SITL'),
        DeclareLaunchArgument('fcu_url', default_value='udp://0.0.0.0:14550@',
                              description='MAVROS nghe SITL ở UDP 14550'),
        DeclareLaunchArgument('gcs_url', default_value='',
                              description='Chuyển tiếp MAVLink cho QGroundControl, ví dụ udp-b://@'),
        DeclareLaunchArgument('teleop', default_value='true', description='Chạy teleop_node'),
        DeclareLaunchArgument('joy', default_value='false', description='Chạy joy_node (tay cầm USB)'),
        DeclareLaunchArgument('safety', default_value='true', description='Chạy safety_node'),
        DeclareLaunchArgument('chase', default_value='true',
                              description='Camera Gazebo tự bám đuôi robot (false = tự điều khiển camera)'),
        DeclareLaunchArgument('chase_distance', default_value='1.3', description='m phía sau robot'),
        DeclareLaunchArgument('chase_height', default_value='0.5', description='m phía trên robot'),
        gazebo, sitl, mavros, safety_node, teleop_node, joy_node, chase_camera,
    ])
