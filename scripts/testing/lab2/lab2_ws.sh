# Sourced by the `just lab2` recipes that start a vehicle model, after scripts/env.sh.
#
#   lab2_ws_source      # the teams' workspace, ~/lab2/ws ($LAB2_HOME/ws), when it is built
#
# Team vehicle models (`just lab2 new-team N`) live in ~/lab2/ws/src and are built there
# (`just lab2 build-teams`) as an overlay over this repository's install, so
# vehicle_model:=teamN_vehicle resolves. Without a built workspace nothing changes: the
# reference model, autosdv_vehicle, is in this repository's install.
lab2_ws_source() {
    local ws="${LAB2_HOME:-$HOME/lab2}/ws"
    if [ -f "$ws/install/local_setup.bash" ]; then
        set +u
        # shellcheck disable=SC1091
        source "$ws/install/local_setup.bash"
        set -u
    fi
}
