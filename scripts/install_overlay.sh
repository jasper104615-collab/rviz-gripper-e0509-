#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DOOSAN_SRC="${DOOSAN_SRC:-$HOME/doosan-robot2/src/doosan-robot2}"
RH12_SRC="${RH12_SRC:-$HOME/rh_p12_rna_controller}"

echo "[1/2] Doosan URDF (팔+그리퍼)"
DST_URDF="$DOOSAN_SRC/dsr_description2/urdf/e0509_with_gripper.urdf"
if [[ ! -d "$(dirname "$DST_URDF")" ]]; then
  echo "ERROR: $DST_URDF not found. Set DOOSAN_SRC."
  exit 1
fi
cp "$BUNDLE_DIR/overlay/doosan/dsr_description2/urdf/e0509_with_gripper.urdf" "$DST_URDF"
echo "  -> $DST_URDF"

echo "[2/2] RH-P12 gripper_node"
DST_GN="$RH12_SRC/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py"
if [[ ! -f "$DST_GN" ]]; then
  echo "ERROR: $DST_GN not found. Set RH12_SRC."
  exit 1
fi
cp "$BUNDLE_DIR/overlay/rh_p12/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py" "$DST_GN"
echo "  -> $DST_GN"
echo "Done. Rebuild dsr_description2 + rh_p12_rna_controller."
