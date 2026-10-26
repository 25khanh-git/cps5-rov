#!/usr/bin/env bash
# =============================================================================
# Cài toàn bộ môi trường sim CPS5 trong WSL2 Ubuntu 22.04.
#
#   bash scripts/setup_wsl.sh            # cài tất cả (30–90 phút, cần ~15 GB trống)
#   bash scripts/setup_wsl.sh ros        # chỉ chạy một bước: base | ros | gazebo | ardupilot | plugin | workspace
#
# Chạy lại an toàn: bước nào đã xong sẽ được bỏ qua. Log: ~/.cps5/setup.log
#
# Cài:
#   1. base      — công cụ cơ bản, locale UTF-8
#   2. ros       — ROS 2 Humble (desktop) + MAVROS + joy + v4l2_camera + colcon
#   3. gazebo    — Gazebo Harmonic + ros_gz cho Humble (ros-humble-ros-gzharmonic)
#   4. ardupilot — mã nguồn ArduPilot tại tag ArduSub ($ARDUSUB_REF), build ArduSub SITL
#   5. plugin    — ardupilot_gazebo (ArduPilotPlugin cho Gazebo Harmonic)
#   6. workspace — build workspace CPS 5 (scripts/build.sh)
# =============================================================================
set -eo pipefail

ARDUSUB_REF="${ARDUSUB_REF:-Sub-4.7.2}"          # phải cùng phiên bản với firmware trên Pixhawk 6C
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
ARDUPILOT_GAZEBO_REF="${ARDUPILOT_GAZEBO_REF:-main}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$HOME/.cps5"
mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_DIR/setup.log") 2>&1

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    ✓ %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m    ✗ %s\033[0m\n' "$*"; exit 1; }
APT="sudo DEBIAN_FRONTEND=noninteractive apt-get"

preflight() {
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${VERSION_CODENAME:-}" = "jammy" ] || die "Cần Ubuntu 22.04 (jammy). Máy này: ${PRETTY_NAME:-?}.
    Cài trong PowerShell (Admin):  wsl --install -d Ubuntu-22.04"
  [ "$EUID" -ne 0 ] || die "Đừng chạy bằng sudo/root. Chạy: bash scripts/setup_wsl.sh"
  if grep -qi microsoft /proc/version 2>/dev/null; then ok "Đang chạy trong WSL"; fi
  local free_gb
  free_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9')
  [ "${free_gb:-0}" -ge 15 ] || echo "    ! Chỉ còn ${free_gb} GB trống trong WSL — có thể không đủ (~15 GB)."
  sudo -v
}

step_base() {
  log "1/6 Công cụ cơ bản"
  $APT update
  $APT install -y curl gnupg lsb-release git build-essential cmake python3-pip \
    software-properties-common locales wget
  sudo locale-gen en_US en_US.UTF-8 >/dev/null
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository -y universe >/dev/null
  ok "base"
}

step_ros() {
  log "2/6 ROS 2 Humble + MAVROS"
  if ! dpkg -s ros2-apt-source >/dev/null 2>&1 && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    local ver
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [ -n "$ver" ] || die "Không lấy được phiên bản ros-apt-source (mạng?)"
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.jammy_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
  fi
  $APT update
  $APT install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
    python3-yaml python3-pytest \
    ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
    ros-humble-joy ros-humble-v4l2-camera ros-humble-rqt-image-view
  # MAVROS cần dữ liệu GeographicLib (geoid) — thiếu thì mavros_node thoát ngay khi khởi động
  if [ ! -d /usr/share/GeographicLib/geoids ] || [ -z "$(ls -A /usr/share/GeographicLib/geoids 2>/dev/null)" ]; then
    local geo=/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    [ -f "$geo" ] || geo="$(find /opt/ros/humble -name install_geographiclib_datasets.sh | head -1)"
    [ -n "$geo" ] || die "Không tìm thấy install_geographiclib_datasets.sh của MAVROS"
    sudo bash "$geo"
  fi
  if ! grep -q "CPS5 env" "$HOME/.bashrc"; then
    {
      echo ""
      echo "# CPS5 env (thêm bởi scripts/setup_wsl.sh)"
      echo "[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash"
    } >> "$HOME/.bashrc"
  fi
  ok "ROS 2 Humble"
}

step_gazebo() {
  log "3/6 Gazebo Harmonic + ros_gz (Humble)"
  if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  fi
  $APT update
  # ros-humble-ros-gzharmonic thay cho ros-humble-ros-gz (bản đó đi với Gazebo Fortress)
  $APT install -y gz-harmonic ros-humble-ros-gzharmonic python3-gz-transport13
  gz sim --versions || true
  ok "Gazebo Harmonic"
}

step_ardupilot() {
  log "4/6 ArduPilot ($ARDUSUB_REF) + ArduSub SITL"
  if [ ! -d "$ARDUPILOT_DIR/.git" ]; then
    git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
  fi
  cd "$ARDUPILOT_DIR"
  git fetch --tags origin
  git checkout -f "$ARDUSUB_REF"
  git submodule update --init --recursive
  # Không cài toolchain STM32 (chỉ cần SITL), không cài GUI MAVProxy
  DO_AP_STM_ENV=0 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 \
    Tools/environment_install/install-prereqs-ubuntu.sh -y
  # Login shell để nhận PATH/PYTHONPATH mà install-prereqs vừa thêm vào ~/.profile
  bash -lc "cd '$ARDUPILOT_DIR' && ./waf configure --board sitl && ./waf sub"
  [ -x "$ARDUPILOT_DIR/build/sitl/bin/ardusub" ] || die "Build ArduSub SITL thất bại"
  ok "ArduSub SITL: $ARDUPILOT_DIR/build/sitl/bin/ardusub"
}

step_plugin() {
  log "5/6 ardupilot_gazebo (ArduPilotPlugin)"
  $APT install -y libgz-sim8-dev rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  if [ ! -d "$ARDUPILOT_GAZEBO_DIR/.git" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$ARDUPILOT_GAZEBO_DIR"
  fi
  cd "$ARDUPILOT_GAZEBO_DIR"
  git fetch origin && git checkout -f "$ARDUPILOT_GAZEBO_REF" && git pull --ff-only || true
  mkdir -p build && cd build
  GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j"$(nproc)"
  [ -f "$ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so" ] || die "Build ArduPilotPlugin thất bại"
  ok "ArduPilotPlugin: $ARDUPILOT_GAZEBO_DIR/build/libArduPilotPlugin.so"
}

step_workspace() {
  log "6/6 Build workspace CPS 5"
  bash "$ROOT/scripts/build.sh"
  if ! grep -q "CPS5 workspace" "$HOME/.bashrc"; then
    {
      echo "# CPS5 workspace (thêm bởi scripts/setup_wsl.sh)"
      echo "[ -f '$ROOT/scripts/env.sh' ] && source '$ROOT/scripts/env.sh'"
    } >> "$HOME/.bashrc"
  fi
  ok "workspace"
}

preflight
if [ $# -gt 0 ]; then
  for s in "$@"; do "step_$s"; done
else
  step_base; step_ros; step_gazebo; step_ardupilot; step_plugin; step_workspace
fi

log "XONG. Mở terminal WSL mới rồi:"
echo "    bash scripts/run_sim.sh          # chạy sim"
echo "    bash scripts/keyboard.sh         # lái bằng bàn phím (terminal khác)"
echo "    bash scripts/check_sim.sh        # kiểm tra tự động 9 mục"
