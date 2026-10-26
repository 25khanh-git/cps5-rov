#!/usr/bin/env bash
# Kiểm tra tự động: bật sim (không cửa sổ, không teleop) -> chạy check_sim -> tắt sim.
#   bash scripts/check_sim.sh            # headless
#   bash scripts/check_sim.sh gui:=true  # xem robot chạy test
# Kết quả chi tiết của sim: ~/.cps5/check_sim_launch.log
set -eo pipefail
source "$(dirname "$0")/env.sh"
if [ ! -f "$CPS5_WS/install/setup.bash" ]; then
  echo "Chưa build. Chạy: bash scripts/build.sh" >&2; exit 1
fi
LOG="$HOME/.cps5/check_sim_launch.log"
mkdir -p "$(dirname "$LOG")"

echo "Đang bật simulation (log: $LOG) ..."
setsid ros2 launch rov_bringup sim.launch.py gui:=false teleop:=false "$@" >"$LOG" 2>&1 &
SIM_PID=$!
cleanup() {
  echo "Đang tắt simulation ..."
  kill -INT -- "-$SIM_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

set +e
ros2 run rov_bringup check_sim
CODE=$?
set -e
if [ "$CODE" -ne 0 ]; then
  echo "Có mục KHÔNG ĐẠT. 40 dòng cuối log simulation:"
  tail -n 40 "$LOG"
fi
exit "$CODE"
