"""Chỉ chạy Gazebo + bridge (không SITL, không MAVROS).

Dùng để xem model / thế giới, hoặc được sim.launch.py của rov_bringup include lại.
  ros2 launch cps5_gazebo gazebo.launch.py gui:=true render_engine:=ogre2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    from cps5_description import model as cps5_model
    from cps5_gazebo import world as cps5_world

    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    engine = LaunchConfiguration('render_engine').perform(context)
    vehicle = LaunchConfiguration('vehicle').perform(context) or None
    world = LaunchConfiguration('world').perform(context) or None
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context))

    world_path = cps5_world.write_world(out_dir, vehicle, world, engine, camera)
    summary = cps5_model.summary(cps5_model.load_params(vehicle))

    ap_gz = os.environ.get('ARDUPILOT_GAZEBO_DIR', os.path.expanduser('~/ardupilot_gazebo'))
    plugin_dir = os.path.join(ap_gz, 'build')
    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            p for p in [plugin_dir, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')] if p),
    }
    actions = [LogInfo(msg=summary), LogInfo(msg=f'World SDF: {world_path}')]
    if not os.path.isfile(os.path.join(plugin_dir, 'libArduPilotPlugin.so')):
        actions.append(LogInfo(msg=(
            f'CẢNH BÁO: không thấy {plugin_dir}/libArduPilotPlugin.so — '
            'chạy scripts/setup_wsl.sh hoặc đặt biến ARDUPILOT_GAZEBO_DIR.')))

    cmd = ['gz', 'sim', '-r', '-v', '3', '--render-engine', engine]
    if not gui:
        cmd += ['-s', '--headless-rendering']  # chỉ server; camera render bằng EGL
    cmd.append(world_path)
    gz = ExecuteProcess(cmd=cmd, additional_env=env, output='screen', name='gz_sim')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        parameters=[{'config_file': os.path.join(
            get_package_share_directory('cps5_gazebo'), 'config', 'bridge.yaml')}],
    )
    # Đóng Gazebo => tắt cả launch
    stop_all = RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown())]))
    return actions + [gz, bridge, stop_all]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Mở cửa sổ Gazebo'),
        DeclareLaunchArgument('camera', default_value='true', description='Bật camera (cần render)'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
                              description='ogre2 (mặc định) hoặc ogre nếu GPU/WSL lỗi'),
        DeclareLaunchArgument('vehicle', default_value='', description='YAML thông số robot'),
        DeclareLaunchArgument('world', default_value='', description='YAML thông số thế giới'),
        DeclareLaunchArgument('out_dir', default_value='~/.cps5/generated',
                              description='Nơi ghi file SDF sinh ra'),
        OpaqueFunction(function=_setup),
    ])
