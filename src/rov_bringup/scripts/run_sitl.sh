#!/usr/bin/env bash
# Chạy ArduSub SITL (firmware ArduSub thật, biên dịch cho PC) nối với Gazebo.
#   - Vật lý: JSON qua UDP 9002 <-> ArduPilotPlugin trong Gazebo (lock-step)
#   - MAVLink: SERIAL0 gửi UDP tới 127.0.0.1:14550, nơi MAVROS đang nghe
# Không dùng sim_vehicle.py / MAVProxy để launch gọn và ít phụ thuộc Python.
#
# Cách dùng: run_sitl.sh <file.parm> [lat,lon,alt,heading]
set -eo pipefail

PARM_FILE="${1:?Thiếu đường dẫn file .parm}"
HOME_LOC="${2:-21.0047,105.8434,0,0}"   # Hà Nội, mặt nước (alt 0)
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
WORK_DIR="${CPS5_SITL_DIR:-$HOME/.cps5/sitl}"
MAVLINK_OUT="${CPS5_MAVLINK_OUT:-udpclient:127.0.0.1:14550}"

BIN="$ARDUPILOT_DIR/build/sitl/bin/ardusub"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/sub.parm"

if [ ! -x "$BIN" ]; then
  echo "[run_sitl] Không thấy $BIN" >&2
  echo "[run_sitl] Chạy scripts/setup_wsl.sh (bước ArduPilot) hoặc đặt biến ARDUPILOT_DIR." >&2
  exit 1
fi
if [ ! -f "$PARM_FILE" ]; then
  echo "[run_sitl] Không thấy file tham số $PARM_FILE" >&2
  exit 1
fi

mkdir -p "$WORK_DIR"
cd "$WORK_DIR"   # eeprom.bin và log SITL nằm ở đây

# SITL cũ còn sót từ lần chạy trước (Ctrl+C, crash) sẽ giữ cổng 5760/5762 và làm SITL mới
# chết ngay ("bind failed"), MAVROS lại nối nhầm vào SITL cũ -> ARM bị từ chối. Tắt nó trước.
if pgrep -f "$BIN" >/dev/null 2>&1; then
  echo "[run_sitl] Phát hiện ArduSub SITL cũ còn chạy -> tắt" >&2
  pkill -f "$BIN" || true
  for _ in $(seq 1 20); do pgrep -f "$BIN" >/dev/null 2>&1 || break; sleep 0.25; done
  pkill -9 -f "$BIN" 2>/dev/null || true
fi

# Tham số chỉ dành cho SITL (SIM_*), không có trên Pixhawk nên không để trong cps5.parm.
# SIM_RC_FAIL 1: tắt bộ thu RC giả lập của SITL. Robot thật không có bộ thu RC, chỉ có
# RC override từ MAVROS; nếu để bộ thu giả, khi override dừng ArduSub quay về RC giả
# (1500) và failsafe mất lệnh lái (FS_PILOT_INPUT) không bao giờ kích hoạt.
SITL_ONLY="$WORK_DIR/sitl_only.parm"
cat > "$SITL_ONLY" <<'EOF'
SIM_RC_FAIL 1
EOF

echo "[run_sitl] $BIN"
echo "[run_sitl] tham số: $DEFAULTS + $PARM_FILE + $SITL_ONLY"
# -w: xóa EEPROM mỗi lần chạy => luôn nạp lại đúng tham số (FRAME_CONFIG cần reboot mới có hiệu lực)
exec "$BIN" \
  --model JSON \
  --speedup 1 \
  -I0 \
  -w \
  --home "$HOME_LOC" \
  --defaults "$DEFAULTS,$PARM_FILE,$SITL_ONLY" \
  --serial0 "$MAVLINK_OUT"
