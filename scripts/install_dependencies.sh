#!/usr/bin/env bash

set -euo pipefail

# Override to pin a tag/commit of PiShrink for reproducible installs.
PISHRINK_REF="${PISHRINK_REF:-master}"

sudo apt update
sudo apt install -y pv systemd-container qemu-user-static parted rsync wget xz-utils zstd e2fsprogs

if ! command -v pishrink.sh >/dev/null; then
  tmp="$(mktemp)"
  wget -qO "$tmp" "https://raw.githubusercontent.com/Drewsif/PiShrink/${PISHRINK_REF}/pishrink.sh"
  sudo install -m 0755 "$tmp" /usr/local/bin/pishrink.sh
  rm -f "$tmp"
fi
