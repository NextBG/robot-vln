#!/usr/bin/env bash
# Shared helpers for the onboard setup scripts.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO_ROOT}/ros2_ws"

info() { echo -e "\033[1;32m[INFO]\033[0m $*"; }
warn() { echo -e "\033[1;33m[WARN]\033[0m $*"; }
die()  { echo -e "\033[1;31m[ERR ]\033[0m $*" >&2; exit 1; }

require_jammy() {
  . /etc/os-release
  [[ "${VERSION_CODENAME:-}" == "jammy" ]] || die "Expected Ubuntu 22.04 (jammy), got ${PRETTY_NAME:-unknown}"
}

append_once() {  # append_once <line> <file>
  grep -qxF "$1" "$2" 2>/dev/null || echo "$1" >> "$2"
}

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
}
