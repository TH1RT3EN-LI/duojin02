#!/usr/bin/env bash
set -euo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_options=()
build_options+=(--build-arg "BUILD_JOBS=${BUILD_JOBS:-2}" --build-arg "BUILD_WORKERS=${BUILD_WORKERS:-1}")
image="${IMAGE:-duojin02:hardware-humble}"
if [[ -n "${PLATFORM:-}" ]]; then
  build_options+=(--platform "$PLATFORM")
  if [[ -z "${IMAGE:-}" && "$PLATFORM" == "linux/arm64" && "$(uname -m)" != "aarch64" ]]; then
    image=duojin02:hardware-humble-arm64
  fi
fi
supplied_arguments=("$@")
for ((argument_index = 0; argument_index < ${#supplied_arguments[@]}; argument_index++)); do
  case "${supplied_arguments[argument_index]}" in
  --target)
    if [[ -z "${IMAGE:-}" && $((argument_index + 1)) -lt ${#supplied_arguments[@]} ]]; then
      image="${image}-${supplied_arguments[argument_index+1]}"
    fi
    ;;
  --target=*)
    if [[ -z "${IMAGE:-}" ]]; then image="${image}-${supplied_arguments[argument_index]#--target=}"; fi
    ;;
  esac
done
for proxy_variable in HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy; do
  if [[ -n "${!proxy_variable:-}" ]]; then
    build_options+=(--build-arg "$proxy_variable")
  fi
done
build_network="${DOCKER_BUILD_NETWORK:-}"
if [[ -z "$build_network" && "$(uname -s)" == "Linux" ]]; then
  proxy_endpoints="${HTTP_PROXY:-}${HTTPS_PROXY:-}${ALL_PROXY:-}${http_proxy:-}${https_proxy:-}${all_proxy:-}"
  if [[ "$proxy_endpoints" == *127.0.0.1* || "$proxy_endpoints" == *localhost* ]]; then
    build_network=host
  fi
fi
if [[ -n "$build_network" ]]; then build_options+=(--network "$build_network"); fi
exec docker build -t "$image" "${build_options[@]}" "$@" "$hardware_root"
