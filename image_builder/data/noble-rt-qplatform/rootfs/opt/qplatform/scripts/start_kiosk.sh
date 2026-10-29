#!/usr/bin/env bash
# ==============================================================================
# QPlatform Wayland Kiosk Launcher (Cage + WebKit/Cog/Chromium)
# ==============================================================================
# Launches a single full-screen browser directly on DRM/KMS (/dev/dri/card*)
# using the `cage` Wayland kiosk compositor. No X11 or desktop environment.
# ==============================================================================

set -e

# Load user configuration overrides if present
if [ -f /etc/default/qplatform-kiosk ]; then
    # shellcheck disable=SC1091
    source /etc/default/qplatform-kiosk
fi

KIOSK_URL="${KIOSK_URL:-file:///opt/qplatform/webui/index.html}"
KIOSK_BROWSER="${KIOSK_BROWSER:-auto}"
KIOSK_ROTATION="${KIOSK_ROTATION:-normal}"

# Ensure XDG_RUNTIME_DIR is valid for Wayland socket creation
if [ -z "${XDG_RUNTIME_DIR:-}" ]; then
    export XDG_RUNTIME_DIR="/run/user/$(id -u)"
fi
if [ ! -d "$XDG_RUNTIME_DIR" ]; then
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 0700 "$XDG_RUNTIME_DIR"
fi

# Force Wayland backend for GTK / WebKit / Ozone
export GDK_BACKEND=wayland
export QT_QPA_PLATFORM=wayland
export MOZ_ENABLE_WAYLAND=1
export WEBKIT_DISABLE_COMPOSITING_MODE="${WEBKIT_DISABLE_COMPOSITING_MODE:-0}"

# Select browser command to run inside cage
if [ "$KIOSK_BROWSER" = "cog" ] || { [ "$KIOSK_BROWSER" = "auto" ] && command -v cog >/dev/null 2>&1; }; then
    BROWSER_CMD=(cog "$KIOSK_URL")
elif [ "$KIOSK_BROWSER" = "chromium" ] && command -v chromium >/dev/null 2>&1; then
    BROWSER_CMD=(chromium --kiosk --noerrdialogs --disable-infobars --ozone-platform=wayland "$KIOSK_URL")
else
    BROWSER_CMD=(/usr/bin/python3 /opt/qplatform/scripts/kiosk_browser.py "$KIOSK_URL")
fi

echo "Starting QPlatform Wayland Kiosk on DRM/KMS..."
echo "  URL     : $KIOSK_URL"
echo "  Command : ${BROWSER_CMD[*]}"

# Run cage (-s allows VT switching with Ctrl+Alt+F1..F6 for debugging, -d disables window decorations)
exec cage -s -d -- "${BROWSER_CMD[@]}"
