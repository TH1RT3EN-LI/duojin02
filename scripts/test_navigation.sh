#!/usr/bin/env bash
# Run only in a container with --network none and no device mappings.
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/source.sh"
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
test_root="${NAVIGATION_TEST_ROOT:-$hardware_root/build/navigation-integration-tests}"
build_jobs="${BUILD_JOBS:-2}"
export CMAKE_BUILD_PARALLEL_LEVEL="$build_jobs"
MAKEFLAGS="-j$build_jobs" colcon --log-base "$test_root/log" build \
  --base-paths "$hardware_root/src/duojin02_navigation" \
    "$hardware_root/src/duojin02_base_driver" "$hardware_root/src/duojin02_bringup" \
  --build-base "$test_root/build" --install-base "$test_root/install" \
  --merge-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$test_root/install/setup.bash"
probe_directory="$(mktemp -d "$test_root/probes-XXXXXX")"
chmod a+rx "$probe_directory"
probe_arguments=(
  --guard-binary "$test_root/install/lib/duojin02_navigation/velocity_guard"
  --driver-binary "$test_root/install/lib/duojin02_base_driver/duojin02_base_driver_node"
)
python3 "$hardware_root/tests/integration/guard_map_validation_probe.py" \
  --guard-binary "$test_root/install/lib/duojin02_navigation/velocity_guard" \
  --params-file "$hardware_root/src/duojin02_bringup/config/safety.yaml" \
  --json-out "$probe_directory/grid-validation.json" --log-dir "$probe_directory/grid-validation-logs"
python3 "$hardware_root/tests/integration/localization_monitor_probe.py" \
  "${probe_arguments[@]}" --output-dir "$probe_directory/localization-monitor"
python3 "$hardware_root/tests/integration/navigation_command_timeout_probe.py" \
  "${probe_arguments[@]}" --params "$hardware_root/src/duojin02_bringup/config/nav2.yaml" \
  --candidate --output-dir "$probe_directory/command-timeout"
echo "Navigation command-chain checks passed; evidence: $probe_directory"
