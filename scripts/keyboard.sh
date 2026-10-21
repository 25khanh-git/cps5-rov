#!/usr/bin/env bash
# Lái robot bằng bàn phím (chạy trong terminal RIÊNG, khi sim đang chạy).
set -eo pipefail
source "$(dirname "$0")/env.sh"
exec ros2 run rov_teleop keyboard_teleop
