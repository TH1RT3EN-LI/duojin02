#!/usr/bin/env bash
duojin02_source_environment() {
  local environment_root
  environment_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return $?
  source "$environment_root/scripts/environment.sh" || return $?
  if [[ "$environment_root" == /opt/duojin02 && -d /ws ]]; then
    environment_root=/ws
  fi
  duojin02_setup_environment runtime "$environment_root"
}

duojin02_source_environment
