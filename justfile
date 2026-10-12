# AutoSDV Development Commands
# Use `just --list` to see all available commands

# ============================================================================
# Modules -- grouped commands. `just <module>` lists that module's recipes.
#
# Invoke either way:  `just bag play`  or  `just bag::play`
#
# Only families entered deliberately live here. The daily verbs (build, test,
# clean, launch, checkout, setup) stay at the root -- partly by choice, and
# partly because a module may NOT share a name with a recipe: `mod launch`
# beside a `launch:` recipe is a hard error that kills the whole justfile.
# ============================================================================

# Demo scenarios: `just demo run` runs the COSS NDT replay end to end
mod demo
# Rosbag recording, playback and download
mod bag 'just/bag.just'
# Control system testing: trajectories and the basic control launch
mod control 'just/control.just'

# The COSS Park scenario: its map, its recording, and the two simulations.
mod coss 'just/coss.just'
# Map validation and occupancy-grid construction
mod map 'just/map.just'
# Simulation: planning simulator, rosbag replay, full scenarios
mod sim 'just/sim.just'
# Development and monitoring tools: RViz, PlotJuggler, TUI, manual control
mod tool 'just/tool.just'
# Lab 2, ArUco localization and waypoint following: bench, vehicle, grading
mod lab2 'just/lab2.just'
# The coach-board pursuit stack (Lab 2 before 2026-10-13), and the field
# calibration recipes (speed steps, motor fit, braking) that both labs use
mod coach 'just/coach.just'
# RC override arbiter firmware (XIAO RP2350): host tests, build, flash
mod arbiter 'just/arbiter.just'

# ============================================================================
# Core Commands
# ============================================================================

# --list-submodules is not optional. Without it each module collapses to a
# single `bag ...` line and its recipes are invisible, which is the difference
# between a menu and a riddle.
# just --list shows only the LAST comment line, so the description goes here.
# Show all available commands, modules expanded
default:
    @just --list --list-submodules

# Initialize and update all git submodules
checkout:
    git submodule update --init --recursive --checkout

# Run interactive setup (installs ROS 2, dependencies, etc.)
setup:
    ./setup.sh

# ============================================================================
# Autoware model data and TensorRT engines
# ============================================================================

# just --list shows only the LAST comment line, so the description goes here.
# Link Autoware model data into data/autoware_data (writable, for TensorRT)
setup-autoware-data:
    ./scripts/setup_autoware_data.sh

# Compile the TensorRT engines this stack needs, ahead of the first launch.
#
# Autoware compiles an .onnx into a .engine inside the NODE'S CONSTRUCTOR the
# first time it runs. On the Orin that is minutes per model, during which the
# node is not up and perception is unavailable. Doing it here turns that into a
# provisioning step.
#
# Engines are specific to the TensorRT version AND the GPU, so this must run ON
# THE TARGET BOARD. It cannot be baked into an image built elsewhere, and it
# must be re-run after an Autoware or JetPack upgrade.
#
# The model set below is what the perception presets actually resolve to:
# centerpoint_tiny (the perception stack resolves the centerpoint model to its
# tiny variant at runtime -- verified from a live launch's TRT input filenames,
# not from the launch arg defaults), the yolox-sPlus camera 2D detector, and
# the three traffic-light models that camera_lidar_fusion adds. Re-derive it if
# a preset changes:
#
#     play_launch resolve autosdv_launch autosdv.launch.yaml \
#       launch_perception:=true perception_preset:=camera_lidar_fusion -o ./tmp/m.yaml
#
# `autoware_shape_estimation` is deliberately absent: it uses TensorRT but ships
# no `build_only` argument, so its pointnet engine is still built on first use.
# That is one model rather than five, and it succeeds now that the directory is
# writable.
#
# just --list shows only the LAST comment line, so the description goes here.
# Compile TensorRT engines ahead of time (minutes; run on the target board)
build-engines:
    #!/usr/bin/env bash
    # No `set -u`: ROS's own setup.bash reads unbound variables and dies under
    # it (AMENT_TRACE_SETUP_FILES). No `set -e` either — one model failing must
    # not hide the rest, and the summary at the end reports what actually
    # landed.
    set -o pipefail
    just setup-autoware-data
    source /opt/ros/humble/setup.bash
    source /opt/autoware/1.5.0/setup.bash
    # An engine built by a different patch of TensorRT than these Debians were
    # compiled against is discarded on the next launch, so building without this
    # is worse than not building at all -- it burns the time and caches nothing.
    source {{justfile_directory()}}/scripts/trt-runtime-env.sh
    DATA="${AUTOSDV_DATA_PATH:-{{justfile_directory()}}/data/autoware_data}"

    # Whether these engines will be REUSED is decided before a single one is
    # built: Autoware discards any plan whose TensorRT version differs from the
    # one its own libraries were compiled against. Read both and compare, rather
    # than scraping the build log -- the same warning line also appears on a
    # legitimate rebuild of a plan left by an older TensorRT, and treating the
    # two alike would cry wolf on exactly the run that fixes the problem.
    TRT_WANT="$(scripts/version/get-version.sh nvidia_amd64.tensorrt_engine_abi 2>/dev/null || true)"
    TRT_GOT="$(scripts/version/trt-loaded-version.sh 2>/dev/null || true)"

    # Each entry: <package> <launch file> [extra args]. `build_only:=true` makes
    # the node exit as soon as its engine is written — an Autoware-provided
    # argument, so the builder settings match what the node will later expect.
    # Building with trtexec by hand would not guarantee that.
    build() {
        local pkg="$1" launch="$2"; shift 2
        echo "=== ${pkg} ${launch}"
        local start=$SECONDS
        # Stream the TensorRT lines rather than piping into `tail`, which
        # buffers the whole build and shows nothing for minutes — on a step
        # that takes minutes per model, silence is indistinguishable from a
        # hang. `--line-buffered` matters for the same reason.
        #
        # Not fatal on failure: one model failing must not hide the others, and
        # the summary below reports what actually landed.
        ros2 launch "${pkg}" "${launch}" data_path:="${DATA}" build_only:=true "$@" 2>&1 \
            | grep --line-buffered -iE "engine generation|engine build|error|fail" \
            | sed -u 's/^/    /' || true
        echo "    (${pkg}: $((SECONDS - start))s)"
    }

    build autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny
    # use_decompress:=false is required: the decompressor node the launch
    # otherwise starts has no build_only and never exits, so `ros2 launch`
    # hangs forever after the engine is written (observed: 3h20m).
    build autoware_tensorrt_yolox yolox_s_plus_opt.launch.xml use_decompress:=false
    build autoware_traffic_light_classifier car_traffic_light_classifier.launch.xml
    build autoware_traffic_light_classifier pedestrian_traffic_light_classifier.launch.xml
    build autoware_traffic_light_fine_detector traffic_light_fine_detector.launch.xml

    echo
    echo "=== engines in ${DATA}"
    find "${DATA}" -name '*.engine' -type f -printf '    %p (%s bytes)\n' 2>/dev/null | sort
    echo "    total: $(find "${DATA}" -name '*.engine' -type f 2>/dev/null | wc -l)"

    # A version-skewed engine loads today and is rebuilt on every launch from
    # here on, which looks exactly like never having run this recipe at all. Say
    # so rather than letting the next person rediscover it.
    if [[ -n "${TRT_WANT}" && -n "${TRT_GOT}" && "${TRT_WANT}" != "${TRT_GOT}" ]]; then
        echo
        echo "=== WARNING: these engines will NOT be reused."
        echo "    This host loads TensorRT ${TRT_GOT}; Autoware was compiled against"
        echo "    ${TRT_WANT}, and autoware_tensorrt_common discards any plan whose"
        echo "    version differs -- so every launch rebuilds all of them. Install the"
        echo "    matching runtime, then re-run this recipe:"
        echo "        ./setup.sh --run --only tensorrt-runtime --yes"
        echo "    See versions.yaml nvidia_amd64.tensorrt_engine_abi."
    fi

# Populate the engine cache from a NEWSLabNTU/AutoSDV release, if this board's
# fingerprint has a match there, then build-engines -- which is the correct
# verify step for free: `build_only:=true` already loads a matching .engine in
# seconds instead of rebuilding it, so a good download is confirmed by how fast
# this finishes, and a bad or missing one just falls through to a real build.
# This is what the tensorrt-engines setup step runs; `just build-engines`
# alone always skips the cache. See docs/roadmap/11-engine-file-delivery.md.
#
# Idempotent and interrupt-safe by construction, not by special-casing:
# - a `.engine-cache-key` marker (this fingerprint, written only after a full
#   sync) makes a second run skip the network entirely when nothing changed --
#   a stale or missing marker just means "try the network again", never a
#   reason to refuse. The marker is believed only while every file the sync
#   put in place is still there (`.engine-cache-files` lists them): a marker
#   beside a deleted engine would otherwise skip the download and pay for a
#   full local build instead.
# - the archive is extracted into a staging directory UNDER `${DATA}` (so `mv`
#   is same-filesystem and atomic), then moved into place file by file, and
#   the marker is written only once every move has succeeded. A kill at any
#   point leaves either the old file or the fully-moved new one at each final
#   path -- never a truncated one -- and never writes a marker for a sync that
#   did not finish, so the next run just tries again rather than trusting a
#   half-applied cache. `build-engines` at the end tolerates a partial mix of
#   old/new/missing engines the same way it always has.
# - the download itself RESUMES. A ~50 MB asset over a conference wifi is a
#   minute of exposure, and starting again from zero after each interruption
#   is how a step that is retried three times still never finishes. The partial
#   file is kept in `.engine-download/` deliberately -- it is the resume point,
#   not litter -- and removed once the sync has succeeded. A resumed file that
#   fails its checksum is re-fetched whole once before being given up on,
#   because that is what a resume onto a changed asset looks like.
# - the manifest is fetched with cache-busting. GitHub's release CDN served a
#   manifest.json 30 seconds stale immediately after an upload during this
#   work; believing it costs a ~9 minute local build for an asset that is
#   sitting on the release.
#
# just --list shows only the LAST comment line, so the description goes here.
# Use a cached engine set if one matches this board, else build (setup.sh default)
engines:
    #!/usr/bin/env bash
    set -o pipefail
    just setup-autoware-data
    DATA="${AUTOSDV_DATA_PATH:-{{justfile_directory()}}/data/autoware_data}"
    REPO="NEWSLabNTU/AutoSDV"
    MARKER="${DATA}/.engine-cache-key"
    SYNCED_FILES="${DATA}/.engine-cache-files"
    DL_DIR="${DATA}/.engine-download"

    # The marker is only as true as the files it claims are in place. A run that
    # trusts it blindly skips the download and then pays for a full local build,
    # which is the expensive way to discover that someone deleted an engine.
    synced() {
        [[ -f "${MARKER}" && "$(cat "${MARKER}" 2>/dev/null)" == "${KEY}" ]] || return 1
        [[ -s "${SYNCED_FILES}" ]] || return 1
        local rel
        while IFS= read -r rel; do
            [[ -n "${rel}" ]] || continue
            [[ -f "${DATA}/${rel}" ]] || return 1
        done < "${SYNCED_FILES}"
        return 0
    }

    if KEY=$(scripts/version/engine-fingerprint.sh 2>&1); then
        if synced; then
            echo "=== ${KEY}: already synced from the cache, skipping download"
        else
            AUTOWARE_VERSION=$(scripts/version/get-version.sh autoware.version)
            TAG="engines-autoware-${AUTOWARE_VERSION}"
            BASE="https://github.com/${REPO}/releases/download/${TAG}"
            echo "=== fingerprint: ${KEY}"

            TMP=$(mktemp -d)
            # A kill during the manifest fetch would otherwise leave this behind
            # in /tmp on every attempt.
            trap 'rm -rf "${TMP}"' EXIT
            # `?t=` and no-cache because the CDN serves a stale manifest for a
            # while after an upload, and a missing key there is indistinguishable
            # from an asset that was never published.
            if curl -fsSL -H 'Cache-Control: no-cache' \
                    "${BASE}/manifest.json?t=$(date +%s)" -o "${TMP}/manifest.json" 2>/dev/null; then
                read -r ASSET ASSET_SHA <<< "$(python3 -c "import json; m = json.load(open('${TMP}/manifest.json')); e = m.get('${KEY}') or {}; print(e.get('asset', ''), e.get('sha256', ''))" 2>/dev/null)"
                if [[ -z "${ASSET:-}" ]]; then
                    echo "=== ${KEY}: no cached engine set for this key yet"
                else
                    mkdir -p "${DL_DIR}"
                    ARCHIVE="${DL_DIR}/${ASSET}"

                    # Three states to handle, in order of cost: already have it
                    # whole, have part of it, have nothing.
                    have_it() {
                        [[ -n "${ASSET_SHA:-}" && -f "${ARCHIVE}" ]] || return 1
                        [[ "$(sha256sum "${ARCHIVE}" | cut -d' ' -f1)" == "${ASSET_SHA}" ]]
                    }

                    # Resume first, then fall back to a whole fetch. Both
                    # attempts are needed, and the SECOND is not optional: a
                    # release URL redirects to a storage host, and a resumed
                    # transfer through that redirect was measured finishing with
                    # curl exit 0 and a file that failed its checksum. Treating
                    # only a curl error as failure sent that case to a ~9 minute
                    # local build with a good asset sitting on the release.
                    for attempt in resume whole; do
                        if have_it; then break; fi
                        if [[ "${attempt}" == "resume" && -f "${ARCHIVE}" ]]; then
                            echo "=== ${ASSET}: resuming interrupted download"
                            curl -fL -C - --retry 3 --retry-delay 2 \
                                -o "${ARCHIVE}" "${BASE}/${ASSET}" 2>/dev/null || true
                        else
                            # No partial to resume, or the resumed file was bad:
                            # start clean rather than resuming onto damage.
                            [[ -f "${ARCHIVE}" ]] && echo "=== ${ASSET}: resumed copy is corrupt, refetching whole"
                            rm -f "${ARCHIVE}"
                            echo "=== ${KEY}: downloading ${ASSET}"
                            curl -fL --retry 3 --retry-delay 2 \
                                -o "${ARCHIVE}" "${BASE}/${ASSET}" 2>/dev/null || true
                        fi
                        # No checksum published for this key: one attempt is all
                        # that can be judged, so take what arrived.
                        [[ -n "${ASSET_SHA:-}" ]] || break
                    done

                    if [[ ! -f "${ARCHIVE}" ]]; then
                        echo "=== ${ASSET}: download failed; building locally instead"
                    elif ! have_it && [[ -n "${ASSET_SHA:-}" ]]; then
                        echo "=== ${ASSET}: checksum mismatch, discarding"
                        rm -f "${ARCHIVE}"
                    else
                        STAGE="${DATA}/.stage-engines"
                        rm -rf "${STAGE}"
                        mkdir -p "${STAGE}"
                        if tar -xzf "${ARCHIVE}" -C "${STAGE}"; then
                            find "${STAGE}" -type f -printf '%P\n' > "${TMP}/files"
                            while IFS= read -r rel; do
                                mkdir -p "${DATA}/$(dirname "${rel}")"
                                mv -f "${STAGE}/${rel}" "${DATA}/${rel}"
                            done < "${TMP}/files"
                            # Both written only now: every file is in place, so
                            # the next run may believe them.
                            cp "${TMP}/files" "${SYNCED_FILES}"
                            echo "${KEY}" > "${MARKER}"
                            # The resume point is no longer needed, and it is
                            # ~50 MB.
                            rm -rf "${DL_DIR}"
                        else
                            echo "=== ${ASSET}: extraction failed, discarding"
                            rm -f "${ARCHIVE}"
                        fi
                        rm -rf "${STAGE}"
                    fi
                fi
            else
                echo "=== no manifest at release ${TAG} (not published yet, or no network)"
            fi
            rm -rf "${TMP}"
            trap - EXIT
        fi
    else
        echo "=== could not fingerprint this board (${KEY}); building locally"
    fi

    just build-engines

# Package this board's already-built engines for a NEWSLabNTU/AutoSDV release.
# Run after `just build-engines`, once per distinct Orin SKU or desktop
# hardware-compat build. Upload the printed asset and fold the printed
# manifest line into that release's manifest.json by hand -- there is no
# maintainer-side release automation yet (phase 3 of the roadmap doc above).
#
# just --list shows only the LAST comment line, so the description goes here.
# Tar this board's engines + print the manifest.json entry for them
export-engines:
    #!/usr/bin/env bash
    set -euo pipefail
    DATA="${AUTOSDV_DATA_PATH:-{{justfile_directory()}}/data/autoware_data}"
    KEY=$(scripts/version/engine-fingerprint.sh)
    ASSET="${KEY}.tar.gz"

    if ! find "${DATA}" -name '*.engine' -type f -print -quit | grep -q .; then
        echo "error: no .engine files under ${DATA} -- run 'just build-engines' first" >&2
        exit 1
    fi

    find "${DATA}" -name '*.engine' -type f -printf '%P\0' \
        | tar -czf "${ASSET}" -C "${DATA}" --null -T -

    SHA=$(sha256sum "${ASSET}" | cut -d' ' -f1)
    echo "=== wrote ${ASSET}"
    echo "=== manifest.json entry:"
    echo "  \"${KEY}\": {\"asset\": \"${ASSET}\", \"sha256\": \"${SHA}\"}"

# --cargo-args --release applies to the Rust packages (cuda_ndt_matcher); without
# it colcon-cargo builds them unoptimized while CMAKE_BUILD_TYPE=Release covers
# only the C++ ones, so pose_source:=cuda_ndt ran a debug binary at ~80 ms per
# scan against the ~5 ms the package documents.

# Build this project
build:
    #!/usr/bin/env bash
    # No CUDA PATH setup needed: cuda_ffi discovers the toolkit by the
    # ecosystem convention (CUDA_PATH/CUDA_HOME, then /usr/local/cuda-*,
    # then the /usr/local/cuda symlink), and ndt_cuda pins cudarc's version
    # features so nothing runs `nvcc --version`. Verified by building with
    # nvcc absent from PATH.
    set -e
    {{justfile_directory()}}/scripts/build/check-python-env.sh
    source /opt/ros/humble/setup.bash && \
    colcon build \
        --base-paths src \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --cargo-args --release

# Run tests for packages in src/ directory
test:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash && \
    colcon test \
        --base-paths src \
        --return-code-on-test-failure; \
    TEST_EXIT_CODE=$?; \
    echo "" && \
    colcon test-result --verbose; \
    exit $TEST_EXIT_CODE

# Clean up built binaries (requires confirmation)
clean:
    #!/usr/bin/env bash
    while true; do \
        read -p 'Are you sure to clean up? (yes/no) ' yn; \
        case $yn in \
            yes ) rm -rf build install log; break;; \
            no ) break;; \
            * ) echo 'Please enter yes or no.';; \
        esac \
    done

# ============================================================================
# Launch Commands - Start systems
# ============================================================================

# Launch AutoSDV system with web UI at http://localhost:8081
launch ARGS="":
    #!/usr/bin/env bash
    # `just launch ARGS="x:=y"` is a spelling this project documented for years
    # and which silently does nothing: in just, ARGS="..." AFTER a recipe name
    # is a positional value, not a variable assignment, so this parameter
    # receives the literal text `ARGS=x:=y` and hands it to play_launch, which
    # cannot use it. The launch then runs with DEFAULT arguments.
    #
    # That is invisible on a machine with an NVIDIA driver -- the default
    # pose_source:=cuda_ndt loads and works -- and fatal on one without, where
    # it panics with no CUDA library to load. Fail loudly instead.
    case "{{ARGS}}" in
        ARGS=*)
            echo "error: write the arguments positionally, without 'ARGS='." >&2
            echo >&2
            echo "    just RECIPE \"pose_source:=ndt launch_perception:=false\"" >&2
            echo >&2
            echo "ARGS= after the recipe name is passed through as a literal" >&2
            echo "launch argument, and the launch silently uses its defaults." >&2
            exit 2
            ;;
    esac
    source scripts/env.sh && \
    if [ -n "$DISPLAY" ]; then \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch autosdv.launch.yaml {{ARGS}}; \
    else \
        play_launch launch \
            --web-addr 0.0.0.0:8081 \
            autosdv_launch autosdv.launch.yaml \
            rviz:=false {{ARGS}}; \
    fi
