#!/usr/bin/env bash
# Integration probes belong in a container with --network none and no device mappings.
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_arguments=(--root "$hardware_root")
if [[ -n "${PREPARED_SLAM_SOURCE:-}" ]]; then
  source_arguments+=(--source "$PREPARED_SLAM_SOURCE")
fi
prepared_source="$(python3 "$hardware_root/tests/integration/slam_test_source.py" "${source_arguments[@]}")"
source "$hardware_root/source.sh"
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
cd "$hardware_root"
test_root="${SLAM_TEST_ROOT:-$hardware_root/build/slam-input-tests}"
cmake -S tests/integration -B "$test_root" -DPREPARED_SLAM_SOURCE="$prepared_source"
cmake --build "$test_root" -j"${BUILD_JOBS:-2}"
python3 tests/integration/ros_isolation.py "$test_root/scan_gate_probe"
python3 tests/integration/ros_isolation.py "$test_root/ceres_anchor_probe"
python3 tests/integration/base_feedback_probe.py
python3 tests/integration/map_save_service_probe.py
python3 tests/integration/recording_probe.py
probe_directory="$(mktemp -d "$test_root/replay-XXXXXX")"
chmod a+rx "$probe_directory"
python3 tests/integration/slam_replay_probe.py "$probe_directory"
echo "SLAM integration checks passed; replay evidence: $probe_directory"
