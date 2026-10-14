#!/usr/bin/env bash
# Explicit Top1 entry point; ordinary navigation.launch.py remains available.
set -e
_top1_dir="${BASH_SOURCE[0]%/*}"
[[ "$_top1_dir" != "${BASH_SOURCE[0]}" ]] || _top1_dir=.
_top1_root="$(cd -- "$_top1_dir/.." && pwd)"
for arg in "$@"; do
  if [[ "$arg" == params_file:=* || "$arg" == chassis_profile:=* ]]; then
    echo "Edit config/chassis.yaml and run scripts/chassis_config.py adopt; navigation uses this sole admitted profile." >&2
    exit 2
  fi
done
source "$_top1_root/source_top1.sh"
exec ros2 launch duojin01_bringup navigation.launch.py \
  "chassis_profile:=$_top1_root/src/duojin01_bringup/config/chassis.yaml" "$@"
