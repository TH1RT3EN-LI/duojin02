#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -eq 1 && ( "$1" == "--help" || "$1" == "-h" ) ]]; then
  exec /usr/bin/python3 "$hardware_root/tests/run_suites.py" --help
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  exec /usr/bin/python3 "$hardware_root/tests/run_suites.py" --docker "$@"
fi
for argument in "$@"; do
  if [[ "$argument" == "--docker" ]]; then
    exec /usr/bin/python3 "$hardware_root/tests/run_suites.py" "$@"
  fi
done
# Validate arguments and existing SLAM source before loading ROS or building.
/usr/bin/python3 "$hardware_root/tests/run_suites.py" --preflight "$@"
source "$hardware_root/scripts/environment.sh"
if [[ -f /.dockerenv && -r /opt/duojin02/source.sh ]]; then
  duojin02_setup_environment runtime /opt/duojin02
else
  duojin02_setup_environment runtime "$hardware_root"
fi
duojin02_setup_build_tools
exec "$DUOJIN02_PYTHON" "$hardware_root/tests/run_suites.py" "$@"
