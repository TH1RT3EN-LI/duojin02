#!/usr/bin/env bash
set -euo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${NAME:-duojin02_hardware}"
if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    docker start "$NAME" >/dev/null
  else
    NAME="$NAME" DETACH=1 "$hardware_root/scripts/dev.sh" >/dev/null
  fi
fi
exec docker exec -it -w /ws -e TERM="${TERM:-xterm-256color}" "$NAME" bash
