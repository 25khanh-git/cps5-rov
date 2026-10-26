#!/usr/bin/env bash
# Sinh lại SDF và in các đại lượng suy ra (thể tích, lực nổi...) — không cần build, không cần ROS.
# Dùng sau khi sửa src/cps5_description/config/cps5_vehicle.yaml.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$HOME/.cps5/generated}"
PYTHONPATH="$ROOT/src/cps5_description:$ROOT/src/cps5_gazebo${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m cps5_gazebo.world --out "$OUT" \
    --vehicle "$ROOT/src/cps5_description/config/cps5_vehicle.yaml" \
    --world "$ROOT/src/cps5_gazebo/config/pool_world.yaml"
