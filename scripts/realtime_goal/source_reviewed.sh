#!/usr/bin/env bash
# Source inside the Humble container. Starts no nodes and makes no hardware calls.
duojin02_source_reviewed_release() {
  local task_root release_root native_prefix backend_prefix prior_colcon_prefix
  task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" || return
  release_root="${DUOJIN_REALTIME_RELEASE_ROOT:-$task_root/output/lilocbench-realtime-goal-implementation-20261019-212448}"
  native_prefix="$release_root/candidate-A03/native/install"
  backend_prefix="$release_root/backend-native/A02/install"
  [[ -r "$native_prefix/local_setup.bash" && -r "$backend_prefix/local_setup.bash" ]] || {
    echo "Reviewed isolated release is unavailable at $release_root" >&2; return 1;
  }
  source /opt/ros/humble/setup.bash || return
  if [[ -r /opt/duojin02/source.sh ]]; then source /opt/duojin02/source.sh || return; fi
  prior_colcon_prefix="${COLCON_CURRENT_PREFIX-}"
  unset COLCON_CURRENT_PREFIX
  source "$backend_prefix/local_setup.bash" || return
  unset COLCON_CURRENT_PREFIX
  source "$native_prefix/local_setup.bash" || return
  if [[ -n "$prior_colcon_prefix" ]]; then export COLCON_CURRENT_PREFIX="$prior_colcon_prefix"; else unset COLCON_CURRENT_PREFIX; fi
  export LD_LIBRARY_PATH="$native_prefix/lib:$backend_prefix/lib:${LD_LIBRARY_PATH:-}"
  export DUOJIN02_WORKSPACE_ROOT="$task_root"
}
duojin02_source_reviewed_release
