#!/usr/bin/env bash
# Build workspace ROS 2 của CPS 5.
#   bash scripts/build.sh                 # build tất cả
#   bash scripts/build.sh --packages-select rov_teleop
# --symlink-install: sửa file Python / launch / YAML trong VS Code là có hiệu lực ngay,
# chỉ cần build lại khi THÊM file mới hoặc sửa setup.py / package.xml.
set -eo pipefail
source "$(dirname "$0")/env.sh"
mkdir -p "$CPS5_WS"
cd "$CPS5_ROOT"
colcon --log-base "$CPS5_WS/log" build \
  --base-paths "$CPS5_ROOT/src" \
  --build-base "$CPS5_WS/build" \
  --install-base "$CPS5_WS/install" \
  --symlink-install "$@"
echo
echo "Build xong. Mở terminal mới hoặc chạy:  source scripts/env.sh"
