#!/usr/bin/env bash
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$hardware_root/scripts/install_dependencies.sh"
echo "Dependencies installed. Run ./scripts/build_hardware.sh and source ./source.sh."
