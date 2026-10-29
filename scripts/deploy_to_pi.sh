#!/usr/bin/env bash
# ==============================================================================
# QPlatform High-Speed Live Deploy Wrapper (Ansible + OpenSSH Multiplexing)
# ==============================================================================
# Runs `ansible/deploy_qplatform.yml` against your Pi CM4 in ~2 seconds.
#
# Usage:
#   ./scripts/deploy_to_pi.sh [PI_IP_OR_HOSTNAME] [PI_USER] [EXTRA_ANSIBLE_ARGS...]
#
# Examples:
#   # Fast deploy to default host (192.168.1.19), prompting once for password:
#   ./scripts/deploy_to_pi.sh -k
#
#   # Deploy ONLY /opt/qplatform (URDF, launch files, scripts, configs):
#   ./scripts/deploy_to_pi.sh -k --tags qplatform
#
#   # Include the 27MB 3D STL meshes directory:
#   ./scripts/deploy_to_pi.sh -k -e sync_meshes=true
#
#   # Full setup including APT package installation & micro-ROS Agent build:
#   ./scripts/deploy_to_pi.sh -k -e install_packages=true
# ==============================================================================

set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$HOME/.local/bin:$PATH"
export ANSIBLE_CONFIG="${REPO_DIR}/ansible/ansible.cfg"
mkdir -p "$HOME/.ansible/cp"

PI_HOST="${1:-}"
PI_USER="${2:-ubuntu}"

if [ -n "$PI_HOST" ] && [[ "$PI_HOST" != -* ]]; then
  shift || true
  if [ -n "${1:-}" ] && [[ "${1:-}" != -* ]]; then
    PI_USER="$1"
    shift || true
  fi
  INVENTORY_ARGS=(-i "${PI_HOST}," -u "${PI_USER}" -e "target_hosts=all")
else
  INVENTORY_ARGS=(-i "${REPO_DIR}/ansible/inventory.ini")
fi

exec ansible-playbook \
  "${INVENTORY_ARGS[@]}" \
  "${REPO_DIR}/ansible/deploy_qplatform.yml" \
  "$@"
