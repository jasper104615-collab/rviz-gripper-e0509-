#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RH12_SRC="${RH12_SRC:-$HOME/rh_p12_rna_controller}"

DST="$RH12_SRC/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py"
if [[ ! -f "$DST" ]]; then
  echo "ERROR: not found: $DST"
  echo "  export RH12_SRC=/path/to/rh_p12_rna_controller"
  exit 1
fi
cp "$BUNDLE_DIR/overlay/rh_p12/rh_p12_rna_controller/rh_p12_rna_controller/gripper_node.py" "$DST"
echo "OK -> $DST"
echo "Rebuild: colcon build --packages-select rh_p12_rna_controller"
