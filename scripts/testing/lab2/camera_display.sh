# Source this, then call lab2_camera_display before a launch that opens the ZED.
#
# The ZED must never see a DISPLAY. Argus builds its buffers through EGL on the
# X server DISPLAY names and fails on any but the car's own desktop with a
# monitor attached: a VNC desktop (:2), the desktop with the monitor unplugged
# (:1), an empty or a dead DISPLAY all stop the ZED at "CAMERA OPENING" with
# "(Argus) Error BadParameter". With no DISPLAY it opens every time.
#
# play_launch keeps a per-node <env>, but drops launch-prefix on a container, so
# the camera cannot be given its own environment. Instead the whole launch runs
# without DISPLAY, and the viewers get theirs back through a viewer_display
# launch argument.
#
# After the call: DISPLAY is unset, VIEWER_DISPLAY holds a working X display for
# viewers or is empty when there is none.
lab2_camera_display() {
    VIEWER_DISPLAY=
    if [ -n "${DISPLAY:-}" ]; then
        local ok=0
        if command -v xdpyinfo >/dev/null 2>&1; then
            xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || ok=1
        elif command -v xset >/dev/null 2>&1; then
            xset -display "$DISPLAY" q >/dev/null 2>&1 || ok=1
        else
            case "$DISPLAY" in
                :*|unix:*) local n=${DISPLAY#*:}; n=${n%%.*}; [ -S "/tmp/.X11-unix/X$n" ] || ok=1 ;;
            esac
        fi
        if [ "$ok" = 0 ]; then
            VIEWER_DISPLAY=$DISPLAY
        else
            echo "warning: DISPLAY='$DISPLAY' has no working X server (a desktop is usually :1 on the car, :2 over VNC); no viewer." >&2
        fi
    fi
    unset DISPLAY
}
