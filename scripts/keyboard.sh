#!/usr/bin/env bash
# Lái robot bằng bàn phím. Chạy trong một terminal riêng, khi sim (hoặc real.launch.py) đang chạy.
set -eo pipefail
source "$(dirname "$0")/env.sh"
exec ros2 run rov_teleop keyboard_teleop
