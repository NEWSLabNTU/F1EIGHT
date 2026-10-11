#!/usr/bin/env bash
# Install/upgrade play_launch (launch orchestrator)
#
# Every `just launch` runs through play_launch, and the floor is 0.15.0. Three
# releases put it there:
#
#   0.9.0   ships the startup governor (MemAvailable admission floor +
#           oom_score_adj on children), which is what stops a full Autoware
#           launch from freezing the box or feeding the desktop to the OOM
#           killer on an edge machine.
#   0.10.0  fixes array parameters being rendered as strings (f78745da). Under
#           0.8.2 that killed autoware_pose_initializer_node at startup and
#           took /localization/initialize with it, so the stack came up without
#           a way to be initialised.
#   0.15.0  passes arguments from an XML include into a YAML launch file, as
#           ros2 launch does (d6aa9d0e), plus the other ros2-launch parity
#           fixes. Lab 2's coach_pursuit.launch.xml relies on it: under 0.14
#           its switches are dropped and map, NDT, perception, planning and
#           stock control start alongside the lab stack.
#
# This is deliberately NOT a marker-gated step, and not part of python-deps:
# a machine set up before the floor carries a python-deps marker and would
# never upgrade past its old play_launch. This script checks the installed
# version on every run and is a no-op when it already satisfies the floor, so
# re-running setup keeps machines current.

set -e

REQUIRED_VERSION="0.15.0"

installed_version() {
    python3 - <<'PY' 2>/dev/null
import importlib.metadata as m
try:
    print(m.version("play_launch"))
except m.PackageNotFoundError:
    pass
PY
}

version_ok() {
    local have="$1"
    [[ -n "$have" ]] || return 1
    [[ "$(printf '%s\n%s\n' "$REQUIRED_VERSION" "$have" | sort -V | head -1)" == "$REQUIRED_VERSION" ]]
}

HAVE="$(installed_version)"

if version_ok "$HAVE"; then
    echo "play_launch ${HAVE} already installed (>= ${REQUIRED_VERSION})."
else
    if [[ -n "$HAVE" ]]; then
        echo "play_launch ${HAVE} is older than ${REQUIRED_VERSION}; upgrading..."
    else
        echo "Installing play_launch (>= ${REQUIRED_VERSION})..."
    fi
    pip3 install --user -U "play_launch>=${REQUIRED_VERSION}"

    HAVE="$(installed_version)"
    if ! version_ok "$HAVE"; then
        echo "ERROR: play_launch ${HAVE:-<none>} installed, need >= ${REQUIRED_VERSION}" >&2
        exit 1
    fi
    echo "play_launch ${HAVE} installed."
fi

if ! command -v play_launch >/dev/null 2>&1; then
    echo
    echo "WARNING: 'play_launch' is not on PATH. It installs to ~/.local/bin;"
    echo "  make sure that directory is in PATH (setup adds it via ~/.profile)."
fi
