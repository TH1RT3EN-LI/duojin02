#!/usr/bin/env bash
# Source this file inside the Humble environment after build_cuda_local.sh.
duojin02_source_cuda_local() {
  local cuda_workspace cuda_variant cuda_prefix
  cuda_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return $?
  case "${DUOJIN_ENABLE_CUDA:-ON}" in
    ON) cuda_variant=cuda ;;
    OFF) cuda_variant=cpu ;;
    *) echo 'DUOJIN_ENABLE_CUDA must be ON or OFF' >&2; return 2 ;;
  esac
  cuda_prefix="$cuda_workspace/install/$cuda_variant-local-v3"
  if [[ ! -r "$cuda_prefix/setup.bash" ||
        ! -r "$cuda_prefix/.target_architecture" ||
        "$(cat "$cuda_prefix/.target_architecture")" != "$(uname -m)" ]]; then
    echo 'Build this architecture first with ./scripts/build_cuda_local.sh' >&2
    return 1
  fi
  source /opt/ros/humble/setup.bash || return $?
  if [[ -r /opt/duojin02/source.sh ]]; then
    source /opt/duojin02/source.sh || return $?
  fi
  source "$cuda_prefix/setup.bash" || return $?
  export LD_LIBRARY_PATH="$cuda_prefix/lib:${LD_LIBRARY_PATH:-}"
  export DUOJIN02_WORKSPACE_ROOT="$cuda_workspace"
}
duojin02_source_cuda_local
