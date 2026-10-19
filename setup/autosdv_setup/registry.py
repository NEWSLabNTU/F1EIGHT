"""Every setup step, in the order they must run.

A step is one object. If it is not in this list it does not run, and if it is in
this list the UI shows it. The previous design split that knowledge three ways --
a MENU_ITEMS table in setup.sh held labels and defaults, the justfile's `setup:`
recipe held the order, and a `_setup-*` wrapper per option held the condition --
so a step nobody wrote a wrapper for ran unconditionally and appeared in no menu.

Dropped from the previous system, and why:

* `pacmod` -- an AutonomouStuff apt source, added with `trusted=yes` so
  signatures are not checked. Nothing under `src/` references pacmod; this
  vehicle's interface is PCA9685 over I2C. Verified by grep before removal.
* `nebula-driver`, `ublox-driver` as apt steps of their own -- `rosdep install
  --from-paths src` already answers them. `ublox_gps` resolves to
  `ros-humble-ublox-gps`, which pulls `ublox-msgs` and `ublox-serialization`
  from one key, and Nebula arrives through `autoware-full-1-5-0`.

Kept, against the golf cart's registry which drops it:

* `gdown` -- still used here, by scripts/2dlidar/download-sample-rosbag.sh and
  cuda_ndt_matcher/scripts/download_sample_data.sh.
* `blickfeld`, `zed-sdk` -- AutoSDV has the hardware that needs them.

`just` is an ordinary step now. It used to be a prerequisite installed by hand
from a piped curl before setup would run at all; setup no longer needs it, but
the rest of the repo does.
"""

from __future__ import annotations

from .model import (
    CHOICE_LABELS, DECLARED, FILES_DIR, HARDWARE_DIR, REPO_ROOT, SCRIPTS_DIR,
    Requires, Step,
)

def _versions() -> dict:
    """versions.yaml, for the few steps whose TEXT needs a version number.

    Hard-coding one here is how `5.1` outlived the wrapper it described. Read
    once, at import; PyYAML is already required by scripts/version/.
    """
    import yaml
    with open(REPO_ROOT / "versions.yaml") as f:
        return yaml.safe_load(f)


_V = _versions()
_ZED_SERIES = _V["zed"]["sdk_series"]
_ZED_SDK = _V["zed"]["sdk_version"]
_JETPACK = _V["nvidia_arm64"]["jetpack"]

_S = lambda name: str(SCRIPTS_DIR / name)          # noqa: E731
_BASH = lambda body: ["bash", "-euc", body]        # noqa: E731


# Preflight fragments. A step that needs something an EARLIER step installs
# must say so itself, because `after` is ordering only -- it never pulls the
# dependency into the selection. `--only tensorrt-engines` on a bare machine,
# or a run whose autoware-debian was unticked, both arrive here with nothing
# installed, and the failure then happens inside a third-party file:
#
#     /opt/ros/humble/setup.bash: No such file or directory
#
# which names neither the step nor the step that would fix it.
_REQUIRE_ROS = """
if [[ ! -f /opt/ros/humble/setup.sh ]]; then
    echo "ROS 2 Humble is not installed: /opt/ros/humble/setup.sh is missing." >&2
    echo "Run the ros2 step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only ros2" >&2
    exit 1
fi
"""

_REQUIRE_AUTOWARE = """
if [[ ! -f /opt/autoware/1.5.0/setup.bash ]]; then
    echo "Autoware 1.5.0 is not installed: /opt/autoware/1.5.0/setup.bash is missing." >&2
    echo "Run the autoware-debian step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only autoware-debian autoware-data" >&2
    exit 1
fi
"""

# The `just` step installs into ~/.local/bin, which Ubuntu's ~/.profile adds to
# PATH only at login and only when the directory already existed. On a first
# run the directory is created mid-pass, so a later step in the same pass
# inherits a PATH without it and `just` is not found. Prepend it rather than
# asking the user to log out and back in between two steps of one run.
_LOCAL_BIN_ON_PATH = """
export PATH="$HOME/.local/bin:$PATH"
if ! command -v just >/dev/null; then
    echo "just is not installed, and this step runs a just recipe." >&2
    echo "Run the just step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --only just" >&2
    exit 1
fi
"""


# Most of src/ is submodules, and a clone that has never run `git submodule
# update` has empty directories there. A step that cds into one then fails on a
# path -- naming neither the submodule nor the step that checks it out.
def _require_submodule(path: str) -> str:
    # Tests for CONTENT, not for `.git`. An uninitialised submodule is an empty
    # directory, which is exactly what this guards against, and emptiness is
    # true wherever the tree came from.
    #
    # `.git` is not: the image build copies the working tree through a context
    # that excludes `**/.git` -- it has to, or the context is gigabytes of
    # history -- so a perfectly good checkout arrives with its files and
    # without its metadata. Testing `.git` failed the range-libc step inside
    # `docker build --target desktop` while the identical tree built fine by
    # hand, which is a miserable thing to debug: the message says "not checked
    # out" about a directory full of source.
    return f"""
if [[ ! -d "{REPO_ROOT}/{path}" ]] || [[ -z "$(ls -A "{REPO_ROOT}/{path}" 2>/dev/null)" ]]; then
    echo "The submodule {path} is not checked out." >&2
    echo "Run the submodules step first, or let a full setup run reach it:" >&2
    echo "    ./setup.sh --run --only submodules" >&2
    exit 1
fi
"""


# rosdep resolves what it FINDS. With submodules missing it reads the handful of
# package.xml files that are in this repository directly, installs their keys,
# and exits 0 -- so the run looks clean and the build fails much later on a
# dependency nobody skipped.
_REQUIRE_SUBMODULES = """
if git submodule status --recursive 2>/dev/null | grep -q '^-'; then
    echo "Some submodules are not checked out, so rosdep would resolve only" >&2
    echo "part of the workspace and still exit 0. Check them out first:" >&2
    echo "    ./setup.sh --run --only submodules" >&2
    exit 1
fi
"""


def _ros_bash(body: str) -> list[str]:
    """Run `body` with ROS 2 Humble sourced.

    Two things this exists for, both learned the hard way.

    `set -u` has to come off around the sourcing. ROS's setup.sh and the ament
    shell hooks read variables that are deliberately unset, so under nounset
    the source aborts with

        /opt/ros/humble/setup.sh: line 124: AMENT_TRACE_SETUP_FILES: unbound
        variable

    which names a variable nobody set, says nothing about the step, and leaves
    `rosdep` looking like the thing that failed. It is restored immediately
    afterwards so the body itself still runs under nounset.

    And ROS may legitimately not be there yet -- see `_REQUIRE_ROS`.
    """
    return ["bash", "-euc", f"""
{_REQUIRE_ROS}
set +u
source /opt/ros/humble/setup.sh
set -u
{body}
"""]

# Only the three declared profiles appear here. `all` and `none` are answered
# by Step.default_for, so nothing has to remember to add a new step to them.
#
# The ladder is dev < vehicle: the vehicle is a development machine that also
# has the sensors and the bus wired to it, so every dev step is a vehicle step
# and the difference is exactly the "System config" group.
DEV = ("dev", "vehicle")
EVERY = ("dev", "vehicle", "ci")
VEHICLE = ("vehicle",)
OPT_IN: tuple[str, ...] = ()            # in no preset; tick it yourself


def _on(*profiles: str) -> dict[str, bool]:
    return {p: (p in profiles) for p in DECLARED}




STEPS: list[Step] = [
    # ---- Workspace -------------------------------------------------------
    Step(
        id="submodules",
        label="Submodule checkout (the source tree itself)",
        why="A fresh clone has empty directories where most of src/ should be. "
            "Later steps then fail on a missing path rather than on a missing "
            "submodule.",
        group="Workspace",
        # Only ever creates. Submodules that are already checked out are left
        # exactly as they are -- including dirty ones and ones parked at another
        # commit, which are ordinary states to be working in and have nothing to
        # do with the empty directories this fills. They are reported, not
        # fixed, and never block the run.
        run=[_S("checkout-submodules.sh")],
        requires=Requires(sudo=False),
        profiles=_on(*EVERY),
        # The state file cannot answer this: `git clean -xfd`, a deleted
        # directory or a fresh clone over the same state all leave a recorded
        # run standing over an empty tree.
        verify=["bash", "-c",
                "! git submodule status --recursive 2>/dev/null | grep -q '^-'"],
    ),

    # ---- Toolchain -------------------------------------------------------
    Step(
        id="just",
        label="just (command runner)",
        why="Every workflow in this repo is a just recipe: build, launch, "
            "record, map checks. Setup itself no longer needs it.",
        group="Toolchain",
        run=_BASH(
            "command -v just >/dev/null && { just --version; exit 0; }; "
            "curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh "
            "| bash -s -- --to \"$HOME/.local/bin\""
        ),
        profiles=_on(*EVERY),
    ),
    Step(
        id="ros2",
        label="ROS 2 Humble",
        why="The base distribution everything else builds against.",
        group="Toolchain",
        run=[_S("install-ros2.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*EVERY),
    ),
    Step(
        id="ros2-dev-tools",
        label="ROS 2 development tools",
        why="colcon, rosdep, vcstool. Needed to build the workspace at all.",
        group="Toolchain",
        run=[_S("install-ros2-dev-tools.sh")],
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*EVERY),
    ),
    Step(
        id="rust",
        label="Rust toolchain (rustup)",
        why="cuda_ndt_matcher is Rust. Without a toolchain colcon skips it and "
            "pose_source:=cuda_ndt has nothing to launch.",
        group="Toolchain",
        run=_BASH(
            "command -v rustup >/dev/null && { rustc --version; exit 0; }; "
            "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"
        ),
        profiles=_on(*EVERY),
    ),
    Step(
        id="colcon-cargo-ros2",
        label="colcon-cargo-ros2 (Rust colcon support)",
        why="Without it colcon skips cuda_ndt_matcher silently and the build "
            "aborts later, on a missing package rather than on this.",
        group="Toolchain",
        run=[_S("install-colcon-cargo-ros2.sh")],
        after=("rust", "ros2-dev-tools"),
        profiles=_on(*EVERY),
        note="Version-floored (>= 0.5.1), so this re-checks on every run.",
    ),
    Step(
        id="play-launch",
        label="play_launch (launch orchestrator)",
        why="Every `just launch` runs through it. 0.9.0 brought the startup "
            "governor; below 0.10.0 the pose initializer's array parameters "
            "render as strings and the node dies at startup; below 0.15.0 "
            "arguments from an XML include never reach a YAML launch file, "
            "which Lab 2's launch relies on.",
        group="Toolchain",
        run=[_S("install-play-launch.sh")],
        profiles=_on(*EVERY),
        note="Version-floored (>= 0.15.0), so this re-checks on every run.",
    ),
    Step(
        id="platformio",
        label="PlatformIO (firmware toolchain)",
        why="Builds and tests the RC override arbiter firmware in "
            "firmware/rc-arbiter (`just arbiter`). The board toolchain itself "
            "downloads into ~/.platformio on the first build, about 500 MB.",
        group="Toolchain",
        run=_BASH(
            "command -v pio >/dev/null && { pio --version; exit 0; }; "
            "if command -v uv >/dev/null; then uv tool install platformio; "
            "elif command -v pipx >/dev/null; then pipx install platformio; "
            "else pip3 install --user platformio; fi"
        ),
        profiles=_on(*DEV),
        verify=["bash", "-c", "command -v pio >/dev/null"],
    ),
    Step(
        id="gdown",
        label="gdown (Google Drive downloader)",
        why="Used by scripts/2dlidar/download-sample-rosbag.sh and the "
            "cuda_ndt_matcher sample-data script.",
        group="Toolchain",
        run=_BASH("pip3 install --user gdown"),
        profiles=_on(*DEV),
    ),
    Step(
        id="dev-tools",
        label="Developer tools (git-lfs, pre-commit, clang-format, PlotJuggler)",
        why="Formatting and the plotting tool `just tool plotjuggler` starts.",
        group="Toolchain",
        run=_BASH(
            "sudo apt-get update && sudo apt-get install -y git-lfs golang && "
            "pip3 install --user pre-commit 'clang-format==17.0.5' && "
            "if ! apt-mark showhold | grep -q ros-humble-plotjuggler-ros; then "
            "sudo apt-get install -y ros-humble-plotjuggler-ros || true; fi"
        ),
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*DEV),
    ),
    Step(
        id="python-deps",
        label="Python dependencies (vehicle control)",
        why="Adafruit-PCA9685 and simple-pid, which the vehicle interface "
            "imports at runtime.",
        group="Toolchain",
        run=_BASH("pip3 install --user Adafruit-PCA9685==1.0.1 simple-pid==2.0.1"),
        profiles=_on(*EVERY),
    ),
    Step(
        id="geographiclib",
        label="GeographicLib + geoid data",
        why="Autoware's map projection needs the egm2008-1 geoid to convert "
            "GNSS altitude.",
        group="Toolchain",
        run=_BASH(
            "sudo apt-get update && sudo apt-get install -y geographiclib-tools && "
            "if [[ ! -f /usr/share/GeographicLib/geoids/egm2008-1.pgm ]]; then "
            "sudo geographiclib-get-geoids egm2008-1; fi"
        ),
        requires=Requires(sudo=True),
        profiles=_on(*EVERY),
    ),

    # ---- Autoware --------------------------------------------------------
    Step(
        id="tensorrt",
        label="TensorRT runtime libraries",
        why="autoware-debian cannot resolve without libnvinfer10, and nothing "
            "else installs it: CUDA and TensorRT come from JetPack on the "
            "vehicle and from the host image on a workstation, so a clean "
            "Ubuntu machine has neither.",
        group="Autoware",
        run=[_S("install-tensorrt.sh")],
        requires=Requires(sudo=True),
        # Ordering is the point: autoware-debian fails without this.
        after=("ros2",),
        profiles=_on(*DEV),
        # A soname check, not a package check, so it is true however TensorRT
        # got here -- network repo, a downloaded local-repo deb, tar or
        # runfile. A machine that already has it is never touched, and the step
        # reads no apt configuration at all.
        # Written to survive `pipefail`, which this does not set but a future
        # caller might: `ldconfig -p | grep -q X` reports FAILURE on a match
        # under pipefail, because grep exits at the first hit and ldconfig then
        # takes SIGPIPE. Capturing once and matching a here-string has no
        # pipeline to misreport, and calls ldconfig once instead of three times.
        verify=["bash", "-c",
                "set -o pipefail; "
                "c=$(ldconfig -p 2>/dev/null) || exit 1; "
                "grep -qF libnvinfer.so.10 <<<\"$c\" && "
                "grep -qF libnvinfer_plugin.so.10 <<<\"$c\" && "
                "grep -qF libnvonnxparser.so.10 <<<\"$c\""],
        note="Installs three runtime libraries only: no CUDA toolkit, no "
             "driver, and /usr/local/cuda is not touched.",
    ),
    Step(
        id="autoware-debian",
        label="Autoware Debian packages",
        why="The 1.5.0 localrepo, about 2-3 GB. Everything in src/ builds "
            "against it.",
        group="Autoware",
        run=[_S("install-autoware-debian.sh")],
        requires=Requires(sudo=True),
        after=("ros2",),
        profiles=_on(*DEV),
    ),
    Step(
        id="autoware-data",
        label="Writable Autoware data directory",
        why="Autoware writes each compiled .engine next to the .onnx it built "
            "from, and the packaged tree is root-owned. Without this every "
            "model fails to write its engine and rebuilds, and fails, on every "
            "launch. Seconds, symlinks only.",
        group="Autoware",
        run=[str(REPO_ROOT / "scripts" / "setup_autoware_data.sh")],
        after=("autoware-debian",),
        profiles=_on(*DEV),
        # Re-run after an Autoware upgrade: the mirror points at the version
        # that was installed when it was built.
        verify=["test", "-d", str(REPO_ROOT / "data" / "autoware_data")],
    ),
    Step(
        id="tensorrt-runtime",
        label="TensorRT runtime matching Autoware's build (amd64)",
        why="Complements the 'tensorrt' step, which installs any TensorRT 10 so "
            "the Autoware packages resolve. This one is about engine REUSE: "
            "Autoware discards any cached .engine whose recorded TensorRT "
            "version differs from the one its own libraries were compiled "
            "against -- to the patch. A workstation whose TensorRT is a "
            "different 10.x patch therefore rebuilds all five perception "
            "engines on EVERY launch (~9 min on an RTX 3090) no matter how "
            "often the engine step has run. This extracts the matching "
            "runtime into /opt/tensorrt/<version>, which only shells that "
            "sourced scripts/env.sh see, so nothing else on the machine "
            "moves. No-op on a Jetson: JetPack already ships the one the "
            "arm64 Debians were built against.",
        group="Autoware",
        run=[_S("install-tensorrt-runtime.sh")],
        requires=Requires(sudo=True, arch=("x86_64",)),
        # After `tensorrt`, which makes the Autoware packages installable with
        # any TensorRT 10 and asks for this exact patch when a source offers it.
        # This step is what rescues the case it cannot fix: a workstation that
        # already had a different 10.x, which is left alone there by design.
        after=("autoware-debian", "tensorrt"),
        profiles=_on(*DEV),
        verify=["bash", "-c",
                'v=$("' + str(REPO_ROOT / "scripts" / "version" / "get-version.sh")
                + '" nvidia_amd64.tensorrt_engine_abi) '
                  '&& test -f "/opt/tensorrt/${v}/lib/libnvinfer.so.${v}"'],
    ),
    # Two ways to get the same five engines, presented as the choice they are:
    # take the published set for this exact board, or build it here. Pick one.
    # Both ticked is not an error -- the download runs first and the build then
    # loads what it finds in seconds -- but it buys nothing either.
    Step(
        id="tensorrt-engines",
        label="Download the published set (build only if none matches)",
        choice="tensorrt-engines",
        why="About 30 s when this board's set is published; builds locally "
            "(~1 hr on an Orin, ~9 min on a desktop GPU) when it is not. The "
            "key is the exact board -- GPU or Orin SKU, TensorRT version, "
            "Autoware version -- and a miss is normal for hardware nobody has "
            "published yet or right after an Autoware upgrade. A download is "
            "verified by loading every engine, so a bad one costs the build it "
            "would have replaced and nothing else. Choose "
            "'tensorrt-engines-build' instead to skip the release entirely.",
        group="Autoware",
        run=_BASH(_LOCAL_BIN_ON_PATH + f'cd "{REPO_ROOT}" && just engines'),
        after=("autoware-data", "just", "tensorrt-runtime"),
        # On by default: without it the same work happens inside the first
        # launch's node constructors instead, where it looks like a hang and
        # leaves perception unavailable until it finishes.
        profiles=_on(*DEV),
        # `.engine-cache-key` records the fingerprint the cache was synced for,
        # and `just engines` re-checks that every file it named is still there,
        # so a deleted engine or an Autoware upgrade shows up as not-installed
        # rather than as a stale tick.
        verify=["bash", "-c",
                f'cd "{REPO_ROOT}" && '
                'k=$(scripts/version/engine-fingerprint.sh 2>/dev/null) && '
                'test "$(cat "${AUTOSDV_DATA_PATH:-./data/autoware_data}/.engine-cache-key" '
                '2>/dev/null)" = "$k"'],
    ),
    Step(
        id="tensorrt-engines-build",
        label="Build here, ignoring the published set",
        choice="tensorrt-engines",
        why="~1 hr on an Orin, ~9 min on a desktop GPU: builds all five "
            "engines here and never asks the release. Must run on the target "
            "board -- an engine is tied to that GPU and that TensorRT. Use it "
            "when a model changed under you, when the published set should not "
            "be trusted, or to produce an asset to publish "
            "(`just export-engines`). Otherwise prefer 'tensorrt-engines', "
            "which lands the same files in about 30 s when someone has "
            "published them for this exact board.",
        group="Autoware",
        run=_BASH(_LOCAL_BIN_ON_PATH + f'cd "{REPO_ROOT}" && just build-engines'),
        after=("autoware-data", "just", "tensorrt-runtime"),
        # Off by default: it is the same outcome as the step above by the
        # expensive route, so it is a deliberate choice, never a default.
        profiles=_on(),
    ),
    Step(
        id="ros-deps",
        label="rosdep install (ROS dependencies of src/)",
        why="Resolves every key the packages under src/ declare, which is also "
            "why there are no per-driver apt steps here. Re-run it after "
            "pulling: new packages bring new keys, and a step that already "
            "ran is skipped.",
        # Its own heading: it must run after autoware-debian, and a step
        # tucked under that group's TensorRT entries was not findable.
        group="Workspace dependencies",
        # `rosdep update` is retried because it fetches the whole rosdistro
        # index from raw.githubusercontent.com, and one unreachable file fails
        # the lot -- including legacy ROS 1 entries this project never uses:
        #
        #   ERROR: unable to process source [.../releases/fuerte.yaml]
        #   ERROR: Not all sources were able to be updated.
        #
        # That is a transient network or rate-limit condition, not a broken
        # workspace, and it should not end an install that has already spent
        # twenty minutes downloading Autoware. Three attempts, then fail for
        # real.
        run=_ros_bash(
            _REQUIRE_SUBMODULES +
            "if [[ -f /opt/autoware/1.5.0/setup.sh ]]; then set +u; "
            "source /opt/autoware/1.5.0/setup.sh; set -u; fi\n"
            f'cd "{REPO_ROOT}"\n'
            "updated=0\n"
            "for attempt in 1 2 3; do\n"
            "  if rosdep update --rosdistro=humble; then updated=1; break; fi\n"
            "  echo \"rosdep update failed (attempt ${attempt}/3); retrying in 15s\" >&2\n"
            "  sleep 15\n"
            "done\n"
            "if [[ $updated -ne 1 ]]; then\n"
            "  echo 'rosdep update failed three times; check network access to "
            "raw.githubusercontent.com' >&2\n"
            "  exit 1\n"
            "fi\n"
            # rosdep resolves keys to apt package NAMES and then shells out to
            # apt-get install; it never refreshes the package lists itself. A
            # machine whose lists are empty or stale therefore fails here with
            #
            #   E: Unable to locate package python3-serial
            #
            # which reads as a missing package rather than a missing index. The
            # container hits this every time, because the layer that installed
            # ROS 2 ends by deleting /var/lib/apt/lists to keep its own size
            # down. range-libc already updates for the same reason.
            "sudo apt-get update\n"
            "rosdep install -y --from-paths src --ignore-src -r"
        ),
        # Read the machine, not the run record: every pull can add keys, so
        # "ran in January" says nothing about today. blickfeld-scanner has no
        # rosdep definition (the blickfeld step installs it), and an
        # unresolvable key makes `rosdep check` exit 2 even when everything
        # resolvable is installed. About 2 s when satisfied.
        verify=_ros_bash(
            "if [[ -f /opt/autoware/1.5.0/setup.sh ]]; then set +u; "
            "source /opt/autoware/1.5.0/setup.sh; set -u; fi\n"
            f'cd "{REPO_ROOT}"\n'
            "rosdep check --from-paths src --ignore-src "
            "--skip-keys blickfeld-scanner >/dev/null 2>&1"
        ),
        requires=Requires(sudo=True),
        after=("submodules", "ros2-dev-tools", "autoware-debian"),
        profiles=_on(*EVERY),
    ),

    # ---- Libraries and drivers ------------------------------------------
    Step(
        id="opencv",
        label="OpenCV consistency (headers, runtime, contrib)",
        why="JetPack leaves 4.8.0 headers over a 4.5.4 runtime, which compiles "
            "and then misbehaves. Also what makes aruco/contrib available.",
        group="Libraries",
        run=[_S("install-opencv.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        verify=["bash", "-c",
                "pkg-config --modversion opencv4 2>/dev/null | grep -q ."],
    ),
    Step(
        id="range-libc",
        label="range_libc (the MCL raycaster)",
        why="pose_source:=mcl raycasts through this Cython extension, and "
            "particle_filter imports it at module scope -- without it that "
            "package cannot even be imported, let alone run.",
        group="Libraries",
        # Built from the submodule rather than pip: it is a Cython extension
        # over the vendored C++ in src/localization/external/range_libc, and
        # there is no wheel of our fork anywhere.
        #
        # cython3 and python3-dev are installed here because they are THIS
        # step's build dependencies and nothing else provides them. Leaving
        # them implicit worked on every machine this was developed on -- all of
        # which already had Cython -- and failed on a clean Ubuntu install with
        #
        #   ModuleNotFoundError: No module named 'Cython'
        #
        # which names the symptom and not the fix. python3-dev is the same
        # class of failure one step later: without Python.h the extension
        # cannot compile.
        run=_BASH(
            _require_submodule("src/localization/external/range_libc") +
            # Keep this check beside the build that needs it.  `just build`
            # performs the same check, but setup can build range_libc directly
            # before colcon ever runs.
            f'"{REPO_ROOT}/scripts/build/check-python-env.sh" && '
            "sudo apt-get update && "
            "sudo apt-get install -y --no-install-recommends cython3 python3-dev && "
            f'cd "{REPO_ROOT}/src/localization/external/range_libc/pywrapper" && '
            "python3 setup.py install --user"
        ),
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # range_libc imports nav_msgs.msg at module scope, so the check has to
        # source ROS. Importing it from a bare python3 fails on nav_msgs and
        # would report the extension as missing when it is present.
        verify=["bash", "-c",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
                "python3 -c 'import range_libc'"],
        after=("submodules",),
        note="Rebuild this after changing the range_libc submodule pin.",
    ),
    Step(
        id="blickfeld",
        label="Blickfeld Scanner Library",
        why="The Cube1 LiDAR driver. Selecting it accepts the library's "
            "licence terms.",
        group="Libraries",
        run=[_S("install-blickfeld.sh")],
        requires=Requires(sudo=True),
        profiles=_on(),                 # opt-in: one of three LiDAR options
    ),
    Step(
        id="zed-sdk",
        label="ZED SDK (checked, not installed -- no apt package exists)",
        why="Checks for the ZED SDK the ROS 2 wrapper is built against and, if "
            "it is missing or the wrong version, prints the exact download for "
            "this machine -- again at the end of the run, highlighted. It "
            "installs nothing: Stereolabs publishes no apt repository, only an "
            "interactive .run installer that asks about a proprietary licence, "
            "which is not something to answer on someone's behalf during "
            "`--yes`. Seconds, and safe to leave selected: a machine with no "
            "ZED loses nothing -- zed_components skips itself and the rest of "
            "the workspace builds.",
        group="Libraries",
        run=[_S("install-zed-sdk.sh")],
        # No sudo: this reads /usr/local/zed and prints. The installer it points
        # at needs root, but the user runs that themselves.
        requires=Requires(sudo=False, network=False),
        profiles=_on(*DEV),
        # The script itself answers, so the version comparison has ONE home --
        # `--check` is the same code path that prints the guidance, minus the
        # printing.
        verify=[_S("install-zed-sdk.sh"), "--check"],
        advice=(
            f"ZED SDK {_ZED_SDK}, which the ROS 2 wrapper in this workspace is\n"
            "built against, is missing or a different version. Cameras will not\n"
            "start; everything else works.\n"
            "\n"
            "Download the installer for THIS machine. These are redirects\n"
            "Stereolabs keeps stable -- the file behind them carries the patch\n"
            "version, so do not bookmark the CDN filename:\n"
            "\n"
            "  amd64, Ubuntu 22.04, CUDA 12\n"
            f"      https://download.stereolabs.com/zedsdk/{_ZED_SERIES}/cu12/ubuntu22\n"
            f"  Jetson, L4T 36.5 (JetPack {_JETPACK} and up -- what this project pins)\n"
            f"      https://download.stereolabs.com/zedsdk/{_ZED_SERIES}/l4t36.5/jetsons\n"
            "  Jetson, L4T 36.4 (JetPack 6.2 / 6.2.1, below the pin)\n"
            f"      https://download.stereolabs.com/zedsdk/{_ZED_SERIES}/l4t36.4/jetsons\n"
            "\n"
            "  curl -fsSL -o zed_sdk.run <url> && chmod +x zed_sdk.run && ./zed_sdk.run\n"
            "\n"
            "Then:  ./setup.sh --rerun zed-sdk     (confirms the version)\n"
            "       just build                     (builds zed_components)"
        ),
    ),

    # ---- System configuration -------------------------------------------
    Step(
        id="cyclonedds-sysctl",
        label="Kernel socket buffers for CycloneDDS",
        why="net.core.rmem_max=2GB plus net.ipv4.ipfrag_*. Below about 10 MB "
            "no ros2 node can start at all.",
        group="System config",
        run=[_S("configure-cyclonedds-sysctl.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # 10 MB is the floor below which no node starts; the step asks for 2 GB.
        verify=["bash", "-c",
                "[[ $(sysctl -n net.core.rmem_max) -ge 10485760 ]]"],
    ),
    Step(
        id="multicast-lo",
        label="Multicast on loopback (persistent)",
        why="cyclonedds.xml pins lo, and lo loses its MULTICAST flag on every "
            "reboot. Installs multicast-lo.service so it survives one.",
        group="System config",
        run=[_S("configure-multicast-lo.sh")],
        requires=Requires(sudo=True),
        profiles=_on(*DEV),
        # lo drops MULTICAST on every reboot unless the unit restores it.
        verify=["bash", "-c", "ip link show lo | grep -q MULTICAST"],
    ),
    Step(
        id="robin-w-ptp",
        label="Seyond Robin-W PTP grandmaster",
        why="Synchronizes the Robin-W timestamp to the Jetson system clock "
            "through the eno1 hardware PTP clock.",
        group="System config",
        run=[_S("configure-robin-w-ptp.sh")],
        requires=Requires(sudo=True, hardware="ptp-nic"),
        profiles=_on(*VEHICLE),
        verify=[
            "bash", "-c",
            "systemctl is-enabled --quiet ptp4l.service && "
            "systemctl is-enabled --quiet phc2sys.service && "
            "systemctl is-active --quiet ptp4l.service && "
            "systemctl is-active --quiet phc2sys.service",
        ],
        note=(
            "After PTP is active, configure the Robin-W with "
            "./setup/files/bin/innovusion_lidar_util "
            "(ptp_en=1, ptp_automotive=0)."
        ),
    ),
    Step(
        id="ublox-udev",
        label="u-blox GNSS udev rules",
        why="Gives the receiver a stable /dev/ublox-gps name and adds you to "
            "dialout. Log out and back in for the group to take effect.",
        group="System config",
        run=_BASH(
            f'sudo cp "{FILES_DIR}/99-ublox-gps.rules" /etc/udev/rules.d/ && '
            "sudo chmod 644 /etc/udev/rules.d/99-ublox-gps.rules && "
            "sudo udevadm control --reload-rules && sudo udevadm trigger && "
            'sudo usermod -aG dialout "$USER" || true'
        ),
        requires=Requires(sudo=True, hardware="ublox-gnss"),
        verify=["test", "-f", "/etc/udev/rules.d/99-ublox-gps.rules"],
        profiles=_on(*VEHICLE),
    ),
    Step(
        id="platformio-udev",
        label="PlatformIO udev rules (flashing the arbiter)",
        why="Lets `just arbiter flash` reach the XIAO RP2350 over USB without "
            "root, and adds you to dialout for `just arbiter monitor`. Log out "
            "and back in for the group to take effect.",
        group="System config",
        run=_BASH(
            "curl -fsSL https://raw.githubusercontent.com/platformio/"
            "platformio-core/develop/platformio/assets/system/99-platformio-udev.rules "
            "| sudo tee /etc/udev/rules.d/99-platformio-udev.rules >/dev/null && "
            "sudo udevadm control --reload-rules && sudo udevadm trigger && "
            'sudo usermod -aG dialout "$USER" || true'
        ),
        requires=Requires(sudo=True),
        after=("platformio",),
        verify=["test", "-f", "/etc/udev/rules.d/99-platformio-udev.rules"],
        profiles=_on(*OPT_IN),
    ),
    Step(
        id="turbovnc-virtualgl",
        label="TurboVNC + VirtualGL",
        why="GPU-accelerated rendering over VNC, which the ZED tools need to "
            "run in a VNC session at all.",
        group="System config",
        run=[_S("install-turbovnc-virtualgl.sh")],
        requires=Requires(sudo=True),
        verify=["bash", "-c", "command -v vglrun >/dev/null"],
        profiles=_on(),                 # opt-in: only for headless boards driven over VNC
    ),
    Step(
        id="turbovnc-xstartup",
        label="TurboVNC desktop session (vehicle)",
        why="TurboVNC's stock xstartup cannot start GNOME while the same user "
            "is logged in on the car's display, so `vncserver` dies at once. "
            "Installs a GNOME --builtin session rendered through VirtualGL "
            "into ~/.vnc, keeping any file it replaces as a .bak, and turns "
            "the screen lock off: unlocking that session hangs.",
        group="System config",
        # The templates are argv so Step.digest() notices when one is edited.
        run=[_S("install-turbovnc-xstartup.sh"),
             str(FILES_DIR / "vnc" / "xstartup.turbovnc"),
             str(FILES_DIR / "vnc" / "turbovncserver.conf")],
        after=("turbovnc-virtualgl",),
        verify=["bash", "-c",
                f"cmp -s {FILES_DIR}/vnc/xstartup.turbovnc ~/.vnc/xstartup.turbovnc"
                f" && cmp -s {FILES_DIR}/vnc/turbovncserver.conf ~/.vnc/turbovncserver.conf"
                " && [ \"$(gsettings get org.gnome.desktop.screensaver lock-enabled)\" = false ]"],
        profiles=_on(),                 # opt-in: a car driven from a laptop over VNC
    ),
    Step(
        id="turbovnc-polkit",
        label="TurboVNC desktop: no Wi-Fi scan password prompts",
        why="The VNC session counts as remote, so polkit asks for a password "
            "on every Wi-Fi scan, every few seconds while Settings is open. "
            "Allows the scan only, for the sudo group.",
        group="System config",
        # argv, not a shell string, so Step.digest() covers the rule file.
        run=["sudo", "install", "-m", "644",
             str(FILES_DIR / "vnc" / "45-autosdv-vnc.pkla"),
             "/etc/polkit-1/localauthority/50-local.d/45-autosdv-vnc.pkla"],
        requires=Requires(sudo=True),
        after=("turbovnc-xstartup",),
        # No verify: 50-local.d is readable by root only.
        profiles=_on(),                 # opt-in, with turbovnc-xstartup
    ),
]

# ---- the `container` profile, derived rather than restated -----------------
# The image used to state its step list in the Dockerfile, as
# `--profile dev --yes --skip cyclonedds-sysctl multicast-lo ros-deps
# range-libc`. That put the list in the wrong file: registry.py is where a step
# is declared, and a `--skip` in a build recipe is invisible to `--list`,
# `--status` and anyone reading this module.
#
# Derived from `dev` membership so a new dev step joins the image without being
# named here -- the same reason `all` and `none` are computed. Only the
# DIFFERENCES are written down, each with the reason it is a difference.
CONTAINER_EXCLUDED = {
    "cyclonedds-sysctl":
        "net.core.rmem_max is not namespaced, so it belongs to whatever kernel "
        "is underneath -- the container cannot set it and `docker run "
        "--sysctl net.core.rmem_max=...` refuses to start at all.",
    "multicast-lo":
        "same: the lo MULTICAST flag is the host's. The entrypoint sets it at "
        "run time when it has NET_ADMIN, and says so when it does not.",
    "ros-deps":
        "reads src/, and the base image deliberately has none. Recoverable on "
        "demand: ./setup.sh --run --only ros-deps range-libc --yes",
    "range-libc":
        "same -- a Cython extension built out of a submodule under src/.",
}

CONTAINER_EXTRA = {
    "turbovnc-virtualgl":
        "the image is driven entirely over noVNC, which is the whole reason a "
        "student on Windows or macOS can see RViz at all. It is opt-in for a "
        "workstation because a workstation has a screen.",
}

# A typo here would silently drop a step from the image, which is the failure
# this file exists to prevent. Fail at import instead.
_declared_ids = {s.id for s in STEPS}
_unknown = (set(CONTAINER_EXCLUDED) | set(CONTAINER_EXTRA)) - _declared_ids
if _unknown:
    raise AssertionError(
        f"container profile names steps that do not exist: {sorted(_unknown)}"
    )

for _step in STEPS:
    _step.profiles["container"] = (
        _step.id in CONTAINER_EXTRA
        or (_step.profiles.get("dev", False) and _step.id not in CONTAINER_EXCLUDED)
    )


BY_ID = {s.id: s for s in STEPS}

# Re-exported: callers look up registry data here, not in model.
__all__ = ["STEPS", "BY_ID", "CHOICE_LABELS", "CONTAINER_EXCLUDED", "CONTAINER_EXTRA",
           "choice_siblings", "collapse_choices", "ordered"]

def choice_siblings(step: Step) -> list[Step]:
    """The alternatives to `step`, itself excluded. Empty for a plain step."""
    if not step.choice:
        return []
    return [s for s in STEPS if s.choice == step.choice and s.id != step.id]


def collapse_choices(selected: set[str]) -> tuple[set[str], list[str]]:
    """Keep at most one step per choice group; return what was dropped.

    Registry order decides the winner, which is the same order the UIs list the
    alternatives in, so the kept one is the one a reader would call first. The
    dropped ids are returned rather than silently discarded: a command line that
    named two alternatives deserves to be told which one is running.
    """
    kept: set[str] = set()
    seen: set[str] = set()
    dropped: list[str] = []
    for step in STEPS:
        if step.id not in selected:
            continue
        if step.choice:
            if step.choice in seen:
                dropped.append(step.id)
                continue
            seen.add(step.choice)
        kept.add(step.id)
    return kept, dropped


def ordered(selected: set[str]) -> list[Step]:
    """Selected steps in registry order, which already respects `after`.

    `after` is declared per step and asserted here rather than used to build a
    graph: the list is short, hand-ordered for readability, and a mismatch is a
    bug worth failing loudly on rather than silently reordering around.
    """
    seen: set[str] = set()
    out: list[Step] = []
    for step in STEPS:
        if step.id not in selected:
            continue
        for dep in step.after:
            if dep in selected and dep not in seen:
                raise AssertionError(
                    f"registry order is wrong: {step.id} runs before {dep}"
                )
        seen.add(step.id)
        out.append(step)
    return out
