#!/usr/bin/env bash
# Nạp môi trường ROS 2 + workspace rồi chạy lệnh (mặc định: real.launch.py).
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
exec "$@"
