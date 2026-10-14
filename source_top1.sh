#!/usr/bin/env bash
# Activate the separately installed, measured Orin release: source ./source_top1.sh
_duojin_top1_activate() {
  local top1_root top1_dir top1_prefix saved_colcon colcon_was_set colcon_was_exported rc runtime_exports setting
  top1_dir="${BASH_SOURCE[0]%/*}"
  [[ "$top1_dir" != "${BASH_SOURCE[0]}" ]] || top1_dir=.
  top1_root="$(cd -- "$top1_dir" && pwd)" || return 1
  top1_prefix="$top1_root/install/orin_top1"
  # Do not activate any overlay before its pinned files and architecture pass.
  LD_PRELOAD= python3 "$top1_root/scripts/verify_orin_top1.py" || return 1
  if [[ ! -r "$top1_prefix/local_setup.bash" ]]; then
    echo "Top1 overlay has no readable local_setup.bash." >&2
    return 1
  fi
  # Clear HIL tracing before setup scripts can launch subprocesses.
  unset DUOJIN_TRACE_DIR HIL_LAYER_SPANS DUOJIN_LAYER_SPANS_DIR DUOJIN_LAYER_OBSERVER_IMPL LD_PRELOAD || return 1
  colcon_was_set="${COLCON_CURRENT_PREFIX+x}"
  saved_colcon="${COLCON_CURRENT_PREFIX-}"
  colcon_was_exported=0
  if [[ "$(declare -p COLCON_CURRENT_PREFIX 2>/dev/null)" == 'declare -x '* ]]; then
    colcon_was_exported=1
  fi
  unset COLCON_CURRENT_PREFIX || return 1
  source "$top1_root/source.sh"
  rc=$?
  if (( rc == 0 )); then
    # local_setup.bash must derive its own prefix, not inherit another workspace's.
    unset COLCON_CURRENT_PREFIX
    source "$top1_prefix/local_setup.bash"
    rc=$?
  fi
  if [[ "$colcon_was_set" == x ]]; then
    COLCON_CURRENT_PREFIX="$saved_colcon"
    if (( colcon_was_exported )); then
      export COLCON_CURRENT_PREFIX
    else
      export -n COLCON_CURRENT_PREFIX
    fi
  else
    unset COLCON_CURRENT_PREFIX
  fi
  (( rc == 0 )) || return "$rc"
  export AMENT_PREFIX_PATH="$top1_prefix/duojin01_navigation${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" || return 1
  export LD_LIBRARY_PATH="$top1_prefix/duojin01_navigation/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" || return 1
  export DUOJIN_CHASSIS_PROFILE="$top1_root/src/duojin01_bringup/config/chassis.yaml" || return 1
  runtime_exports=$(LD_PRELOAD= python3 "$top1_root/scripts/chassis_config.py" env) || return 1
  [[ -n "$runtime_exports" ]] || return 1
  while IFS= read -r setting; do
    export "$setting" || return 1
  done <<< "$runtime_exports"
}
if _duojin_top1_activate; then
  unset -f _duojin_top1_activate
else
  _duojin_top1_status=$?
  unset -f _duojin_top1_activate
  return "$_duojin_top1_status" 2>/dev/null || exit "$_duojin_top1_status"
fi
