#!/usr/bin/env bash
# 공식 Doosan / RH-P12 repo 위에 overlay 파일 복사
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DOOSAN_SRC="${DOOSAN_SRC:-$HOME/doosan-robot2/src/doosan-robot2}"
RH12_SRC="${RH12_SRC:-$HOME/rh_p12_rna_controller}"

echo "[1/2] Doosan URDF overlay"
DST_URDF="$DOOSAN_SRC/dsr_description2/urdf/e0509_with_gripper.urdf"
if [[ ! -d "$(dirname "$DST_URDF")" ]]; then
  echo "ERROR: Doosan dsr_description2 not found: $(dirname "$DST_URDF")"
  echo "  export DOOSAN_SRC=/path/to/doosan-robot2/src/doosan-robot2"
  exit 1
fi
cp "$BUNDLE_DIR/overlay/doosan/dsr_description2/urdf/e0509_with_gripper.urdf" "$DST_URDF"
echo "  -> $DST_URDF"

echo "[2/2] RH-P12 gripper_node overlay (DRL 재시도 + sim2real TCP)"
DST_GRIPPER="$RH12_SRC/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py"
if [[ ! -f "$DST_GRIPPER" ]]; then
  echo "ERROR: RH-P12 gripper_node.py not found: $DST_GRIPPER"
  echo "  export RH12_SRC=/path/to/rh_p12_rna_controller (package root)"
  exit 1
fi
cp "$BUNDLE_DIR/overlay/rh_p12/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py" "$DST_GRIPPER"
echo "  -> $DST_GRIPPER"

echo "Done. Rebuild:"
echo "  cd \$DOOSAN_WS && colcon build --packages-select dsr_description2"
echo "  cd \$RH12_WS   && colcon build --packages-select rh_p12_rna_controller"
