#!/usr/bin/env bash
# Chạy simulation đầy đủ. Tham số thêm được chuyển cho launch, ví dụ:
#   bash scripts/run_sim.sh gui:=false
#   bash scripts/run_sim.sh render_engine:=ogre
set -eo pipefail
source "$(dirname "$0")/env.sh"
if [ ! -f "$CPS5_WS/install/setup.bash" ]; then
  echo "Chưa build. Chạy: bash scripts/build.sh" >&2; exit 1
fi
exec ros2 launch rov_bringup sim.launch.py "$@"
