# shellcheck shell=bash
# Nạp môi trường cho dự án CPS 5. Dùng:  source scripts/env.sh
# (Các script khác trong thư mục này tự gọi file này.)

CPS5_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CPS5_ROOT
# Build/install đặt trong ổ Linux của WSL (nhanh hơn nhiều so với /mnt/d), mã nguồn vẫn ở D:
export CPS5_WS="${CPS5_WS:-$HOME/.cps5/ws}"
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
export ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
export GZ_VERSION=harmonic

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[env] Chưa cài ROS 2 Humble. Chạy: bash scripts/setup_wsl.sh" >&2
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$CPS5_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$CPS5_WS/install/setup.bash"
fi

case ":${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:" in
  *":$ARDUPILOT_GAZEBO_DIR/build:"*) ;;
  *) export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" ;;
esac
