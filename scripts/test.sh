#!/usr/bin/env bash
# Chạy unit test logic thuần (không cần simulation, không cần build).
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$ROOT/src/cps5_description:$ROOT/src/cps5_gazebo:$ROOT/src/rov_teleop:$ROOT/src/rov_safety:$ROOT/src/rov_bringup${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q "$ROOT/src/cps5_description/test" "$ROOT/src/rov_teleop/test" \
  "$ROOT/src/rov_safety/test" "$ROOT/src/rov_bringup/test" "$@"
