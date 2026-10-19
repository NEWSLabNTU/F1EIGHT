#!/usr/bin/env bash
# Install the vehicle's TurboVNC session files into ~/.vnc:
#
#   xstartup.turbovnc      starts GNOME; TurboVNC's own cannot while the same
#                          user is logged in on the car's display (its header
#                          says why)
#   turbovncserver.conf    points vncserver at it -- TurboVNC 3.x never reads
#                          ~/.vnc/xstartup.turbovnc unless told to
#
# and turns off GNOME's screen lock. The VNC session runs GNOME outside GDM, and
# the unlock dialog authenticates through GDM, so a locked VNC desktop takes the
# password and hangs forever. The setting is per user, so the car's own desktop
# loses its lock too.
#
# An existing file that differs is kept as <name>.bak.<time>, not overwritten.
#
# Usage:
#   install-turbovnc-xstartup.sh XSTARTUP_TEMPLATE CONF_TEMPLATE

set -euo pipefail

XSTARTUP_SRC=${1:?usage: install-turbovnc-xstartup.sh XSTARTUP_TEMPLATE CONF_TEMPLATE}
CONF_SRC=${2:?usage: install-turbovnc-xstartup.sh XSTARTUP_TEMPLATE CONF_TEMPLATE}
VNC_DIR="${HOME}/.vnc"

if [[ ! -x /opt/TurboVNC/bin/vncserver ]]; then
    echo "TurboVNC is not installed. Run the turbovnc-virtualgl step first:" >&2
    echo "    ./setup.sh --run --only turbovnc-virtualgl" >&2
    exit 1
fi

mkdir -p "$VNC_DIR"
chmod 700 "$VNC_DIR"

install_file() {        # SRC DEST MODE
    local src=$1 dest=$2 mode=$3
    if [[ -f "$dest" ]] && cmp -s "$src" "$dest"; then
        echo "$dest is already current."
        return
    fi
    if [[ -f "$dest" ]]; then
        local backup
        backup="${dest}.bak.$(date +%Y%m%d-%H%M%S)"
        mv "$dest" "$backup"
        echo "Kept the previous file as $backup"
    fi
    install -m "$mode" "$src" "$dest"
    echo "Installed $dest"
}

install_file "$XSTARTUP_SRC" "${VNC_DIR}/xstartup.turbovnc" 755
install_file "$CONF_SRC" "${VNC_DIR}/turbovncserver.conf" 644

# gsettings writes through dconf's D-Bus service; without a session bus it would
# write to a throwaway in-memory backend and report success.
if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" && -S "/run/user/$(id -u)/bus" ]]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
fi
gsettings set org.gnome.desktop.screensaver lock-enabled false
if [[ "$(gsettings get org.gnome.desktop.screensaver lock-enabled)" != false ]]; then
    echo "Could not turn off the screen lock (no session bus?). Run this step from a" >&2
    echo "terminal on the car's desktop." >&2
    exit 1
fi
echo "Screen lock off (org.gnome.desktop.screensaver lock-enabled false)."

echo ""
[[ -f "${VNC_DIR}/passwd" ]] || \
    echo "Set a VNC password first:  /opt/TurboVNC/bin/vncpasswd"
echo "Start a desktop on :2 (port 5902):  vncserver :2"
echo "Stop it:                            vncserver -kill :2"
echo "Inside it, run OpenGL apps on the GPU with:  vglrun -d :1 <app>"
