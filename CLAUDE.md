# CLAUDE.md

Guidance for Claude Code when working with this repository.

## Project Overview

AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware. Supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) for small-scale autonomous vehicles.

## Versioning

AutoSDV uses [Semantic Versioning](https://semver.org/): `MAJOR.MINOR.PATCH[-PRERELEASE]`

### Single Source of Truth: `versions.yaml`

All version information is centralized in `versions.yaml` at the repo root:
- **AutoSDV version**: Project version, prerelease suffix, release channel
- **Autoware version**: Pinned base version, the Debian filenames, install prefix
- **ROS**: Distribution, installation type, RMW implementation
- **NVIDIA platform**: the target JetPack; the CUDA/cuDNN/TensorRT triples are
  **declared but empty** — see below
- **Tool versions**: clang-format, etc.
- **Package checksums**: SHA256 for the Autoware Debian packages

**Every key `export-versions.sh` reads must exist here.** It indexes the YAML
directly, so one missing key raises `KeyError` and the whole export produces
nothing — while still exiting 0, because the failing Python runs inside a
command substitution. A `source` that silently sets no variables is the failure
mode to watch for; if `$AUTOSDV_VERSION` is empty after sourcing, a key is
missing, not mistyped.

**The NVIDIA versions are not installed by `setup.sh`.** CUDA, cuDNN and
TensorRT come from the host image — JetPack on a Jetson, distribution packages
on an amd64 workstation — so these keys record what the Autoware build targets,
not something this repository provisions. The amd64 triple is empty on purpose:
the values previously documented here (CUDA 12.3, TensorRT 8.6) match no machine
this has been built on, and nothing in the repo pins them. The amd64 TensorRT is
the exception, and it is exact: see `nvidia_amd64.tensorrt_engine_abi` and the
engine-reuse note above.

The arm64 floor is **JetPack 6.2.2 or newer**, which is **L4T 36.5** — NVIDIA's
patch numbering crosses an L4T minor inside the 6.2 series (6.2 → 36.4.3,
6.2.1 → 36.4.4, 6.2.2 → 36.5.0, 6.2.3 → 36.5.2). `nvidia_arm64.jetpack` is a
minimum, not an exact pin, and `nvidia_arm64.l4t` is the thing to compare against
a board's `/etc/nv_tegra_release`.

The Debian suffix stays `jetpack62` — it names the 6.2 *series*
`install-autoware-debian.sh` downloads, not a patch, so do not rename it when a
board moves forward within that series.

Three things follow from the 36.4 → 36.5 crossing, and all three are already
wired to `versions.yaml` rather than hard-coded: the ZED SDK installer is
published per L4T minor (`l4t36.5`), NVIDIA's Jetson apt pocket is `r36.5`, and a
published TensorRT engine set is keyed by L4T — so the AGX Orin asset built on
JetPack 6.2.1 (`orin-agx-orin-R36.4.4-...`) **misses** on a 6.2.2 board and the
engines build locally until a 36.5 set is published.

### Version Helper Scripts

```bash
# Get a specific version value
./scripts/version/get-version.sh autosdv.version      # Returns "0.2.0"
./scripts/version/get-version.sh autoware.version     # Returns "1.5.0"
./scripts/version/get-version.sh autoware.deb_amd64   # Returns the .deb filename

# Export all versions as environment variables (bash; BASH_SOURCE-based)
source ./scripts/version/export-versions.sh
echo $AUTOSDV_VERSION    # 0.2.0
echo $AUTOSDV_CHANNEL    # development
echo $AUTOWARE_VERSION   # 1.5.0
echo $JETPACK_VERSION    # 6.2
```

### Version Bumping Guidelines

| Change Type | Version Bump |
|-------------|--------------|
| Breaking changes (vehicle interface, sensor configs, launch API) | MAJOR |
| Autoware base upgrade | MAJOR |
| New sensor/feature support | MINOR |
| New launch parameters | MINOR |
| Bug fixes, parameter tuning | PATCH |
| Documentation only | PATCH |

### Branch Strategy

| Branch | Version | Channel |
|--------|---------|---------|
| `main` | `X.Y.Z` (stable) | `stable` |
| `develop` | `X.Y.Z-dev` | `development` |
| `release/X.Y` | `X.Y.Z-rc.N` | `stable` |

`main` is the stable branch; day-to-day work goes to `develop`. Feature
branches start from `develop` and merge back into it; `develop` merges into
`main` for a release.

### Branch Protection (GitHub rulesets)

Both `main` and `develop` are protected, and the rules change how you merge:

| Rule | `main` | `develop` |
|------|--------|-----------|
| Pull request required | yes | no (direct push allowed) |
| Linear history required | yes | yes |
| Force push blocked | yes | yes |
| Branch deletion blocked | yes | yes |
| Allowed merge methods | squash, rebase | — (no merge commits either way) |

**Linear history means no merge commits on either branch.** Rebase instead of
merging when you bring in upstream changes:

```bash
# update a feature branch onto develop
git checkout feature/my-thing
git fetch origin
git rebase origin/develop

# update develop itself
git checkout develop
git pull --rebase origin develop
```

Set `git config pull.rebase true` in this repo so `git pull` never creates a
merge commit by accident.

**Merging to `main`** requires a pull request (approvals are not required, but
the PR itself is). Merge it with squash or rebase — the "Create a merge commit"
button is rejected by the ruleset:

```bash
git checkout -b release/X.Y develop     # or push develop and open the PR from it
gh pr create --base main --head develop --title "Release X.Y.Z"
gh pr merge --rebase                    # or --squash; never --merge
```

Prefer `--rebase` for a release PR so each commit from `develop` keeps its own
identity on `main`, and `--squash` for a single-topic PR that should land as one
commit.

**Merging to `develop`** does not need a PR, so a rebased feature branch can be
fast-forwarded in directly:

```bash
git checkout develop
git merge --ff-only feature/my-thing
git push origin develop
```

If the fast-forward is refused, `develop` moved on — rebase the feature branch
again rather than creating a merge commit. PRs into `develop` are still welcome
for review; merge them with squash or rebase for the same reason.

## Submodule Workflow

This workspace is mostly submodules, and two rules keep them from drifting.

### Getting them checked out

`./setup.sh` does it, as its first step (`submodules`), which is there because
forgetting to init the submodules is the first thing anyone new to this
repository does. It fills in the empty ones and **only** the empty ones: a
submodule that is already checked out is left exactly as it is, dirty or parked
at another commit, and is listed at the end rather than moved or complained
about. Nothing about this step blocks a run.

`just checkout` is the blunt version — `git submodule update --init --recursive
--checkout`, which does move a submodule that is sitting off the pin.

### Lockstep: push the submodule first, then the pin

A superproject pin is a commit hash. A hash that exists only in a local
submodule checkout is a pin nobody else can resolve: their `git submodule
update` fails, and CI fails with it. So the order is never negotiable:

```bash
# 1. commit and push inside the submodule
cd src/sensor_component/external/seyond_ros_driver
git checkout autosdv-1.5.0        # never commit on a detached HEAD
git commit -am "..."
git push origin autosdv-1.5.0

# 2. only then record the new pin in the superproject
cd -
git add src/sensor_component/external/seyond_ros_driver
git commit -m "Bump the Seyond driver (...)"
git push origin develop
```

Nested submodules repeat this innermost-first. `seyond_ros_driver` contains
`seyond_sdk`, so a change reaching into the SDK is three pushes in order: SDK,
driver, superproject.

Before committing a pin, check that nothing is uncommitted underneath — a `+`
in this output means the working tree is ahead of the recorded pin:

```bash
git submodule status --recursive | grep '^+'
```

A related habit worth keeping: **read a config out of git, not out of a
submodule working tree.** A working tree can be ahead of, behind, or unrelated
to what the pin actually builds, so a conclusion drawn from it can be about code
that no one else has.

### Two kinds of submodule, two pinning conventions

**Upstream, unforked.** Pinned to a tag or a commit that names a stable release.
There is no branch to follow — following one would silently move the pin — so
these need no `branch` line in `.gitmodules`. Example: `zed-ros2-wrapper` at
`humble-v5.0.0`.

**Our forks** (`NEWSLabNTU/*`, `jerry73204/*`). These carry patches rebased onto
an upstream stable version or `main`, and the patch series lives on a tracking
branch. Record that branch in `.gitmodules`:

```
[submodule "src/localization/external/particle_filter"]
	path = src/localization/external/particle_filter
	url = https://github.com/NEWSLabNTU/particle_filter.git
	branch = autosdv
```

Without the `branch` line the fork's own branch structure is undiscoverable from
this repo: the pin still resolves, but nothing says which branch to commit to,
which branch to rebase, or which branch a rename would strand.

Branch naming follows the Autoware release the patches are current for —
`autosdv-1.5.0`, previously `autosdv-2025.02` and `autosdv-0.45.1`. Keep the old
branches after a rebase; they are the record of what worked against that
release.

When upstream releases, the fork is **rebased** onto the new tag rather than
merged, so the patch series stays a readable list of what we changed and why.

## Essential Commands

### Build & Run
```bash
./setup.sh              # Interactive setup (ROS 2, dependencies)
./setup.sh --status     # Check installation status
just build              # Build all packages
just test               # Run tests
just launch vehicle_id:=car_b   # Launch system on that car (web UI: http://localhost:8081)
just launch "vehicle_id:=car_b ..."  # Launch with parameters (vehicle_id is required)
just clean              # Remove build artifacts
just checkout           # Update git submodules
just setup-autoware-data  # Writable model tree at data/autoware_data (TensorRT needs it)
just build-engines      # Pre-compile TensorRT engines (minutes; run on the target board)
just --list             # Show all available commands
```

Grouped commands live in justfile modules under `just/`, so the top-level list
stays short: `arbiter`, `bag`, `control`, `map`, `sim`, `tool`, plus `demo` in `demo/`.
`just <module>` lists that module's recipes, and both spellings work —
`just bag play` and `just bag::play`.

Two mechanics worth knowing before adding one:

- **A module may not share a name with a recipe.** `mod launch` beside a
  `launch:` recipe is a hard error that kills the whole justfile, which is why
  the daily verbs stay at the root.
- **`just <module>` runs the module's FIRST recipe**, so each module file opens
  with a private `default` that only lists itself. Without it, `just bag` would
  start recording.

A module file also needs `set working-directory := '..'`; otherwise its recipes
run with the cwd set to `just/` and every `./scripts/...` path breaks.

### Demos

Scenarios that run end to end from one command, data preparation included.
They live in a justfile module (`demo/justfile`) so the top-level list stays short.

```bash
just demo               # list the demos
just demo check        # are the prerequisites in place?
just demo run          # COSS NDT replay: fetch data, launch, seed pose, replay, report
just demo stop         # stop the stack it leaves running for inspection
just demo report       # metrics for the most recent run
```

### Tools
```bash
just tool rviz          # Launch RViz
just tool plotjuggler   # PlotJuggler visualization
just tool controller    # Keyboard manual control
just tool tui           # Drive monitor TUI (pose, speed, states)
just tool zed           # ZED camera node alone, for camera testing
```

### Diagnostics
```bash
# live, while a replay or drive is running
python3 scripts/testing/localization/check_ndt_activated.py      # activated, or just alive?
python3 scripts/testing/localization/ndt_quality_report.py       # pose quality, not NVTL
python3 scripts/testing/localization/ndt_alignment_report.py     # scan-to-map residual
python3 scripts/testing/localization/ndt_timeseries.py -o tmp/ndt  # every signal vs time
python3 scripts/testing/localization/check_imu_velocity.py       # the EKF's two inputs

# Jetson resource accounting
scripts/profiling/jetson_gpu_sampler.py -o tmp/gpu.csv           # NVML is absent on Tegra
scripts/profiling/kernel_cpu_report.sh                           # softirq, UDP, DDS cost
```
`scripts/testing/localization/README.md` and `scripts/profiling/README.md` list
the offline counterparts and say what each number means.

### Control Testing
```bash
just control basic      # Launch vehicle control test
just control straight   # Run 10m straight trajectory
just control circle     # Run circular trajectory
```

### Maps
```bash
just map check <map_dir> [pose_source]   # validate a map dir for a method
just map grid-from-pcd <map_dir>         # prints z distribution + suggested band, refuses to guess
just map grid-from-pcd <map_dir> --z-min A --z-max B   # writes the grid + autosdv_map.yaml
just map grid-from-bag <bag> <map_dir>   # accumulate scans instead, for a site with no PCD
```
`pose_source:=mcl` needs an occupancy grid rather than a PCD; `just map check`
verifies the grid is in the lanelet2 map's frame, which is the failure class
that otherwise costs days. See `docs/design/map-handling-per-localization-method.md`.

### Rosbag
```bash
just bag record         # Record outdoor sensor topics
just bag play           # Play most recent recording
```

### Simulation
```bash
just sim planning       # Autoware planning simulator (no sensors needed)
just sim logging        # Logging simulation (rosbag replay)
just sim coss-park      # Full COSS Park simulation scenario
just bag download       # Download test rosbag (~2.8 GB)
```
```bash
just sim rc             # RC car simulator: Autoware + the real vehicle interface (hardware:=sim) + plant
just sim grade DIR      # Lab 2 Part 1 grade: headless, deterministic (--view replays it in RViz)
just sim make-track     # size the graded straight from the speed profile
just lab2 sim                             # Lab 2 Part 1: the one ~/lab2/*_lab_2_1 SpeedPID on 3 RC-sim cars, mark / 25
just lab2 run team:=3                     # Lab 2 Part 2: ~/lab2/team3 on the car, or the vehicle bench + gate elsewhere
just lab2 stack car_b                     # the Lab 2 stack (waypoints + ArUco) on car B, no lab settings; car_a sensor_suite:=vlp16_zed ...
```
The RC simulator's vehicle model, grader and limits: `docs/guides/rc-sim.md`.
Lab 2 Part 1 (the course, the 9 checks, the controller and its physics): `docs/guides/lab2-part1-sim.md`.
See `docs/guides/simulation_testing.md` for the full simulation guide
(planning sim, rosbag replay, CARLA integration).

### Manual Build
```bash
source install/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or use justfile
just build

# Build specific package (must include all standard flags)
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

**Important**: Always use `--base-paths src` and other standard flags from `just build` when running colcon commands manually.

## Architecture

### Core Structure
```
src/
├── launcher/autosdv_launch/
│   ├── launch/                     # Main launch files
│   │   ├── autosdv.launch.yaml     # Main launch (real hardware)
│   │   ├── logging_simulation.launch.yaml  # Simulation/replay mode
│   │   └── autosdv_autoware.launch.xml     # Autoware wrapper
│   └── config/                     # Configuration files
│       ├── perception/preset/      # Perception presets
│       │   ├── lidar_only_preset.yaml
│       │   ├── camera_lidar_fusion_preset.yaml
│       │   └── minimal_preset.yaml
│       ├── localization/preset/    # Localization presets
│       │   ├── default_preset.yaml
│       │   └── eagleye_preset.yaml
│       ├── control/                # Control parameters
│       ├── localization/           # Localization parameters (NDT, EKF, etc.)
│       └── perception/             # Perception parameters
├── localization/                   # Localization packages
├── param/autoware_individual_params/  # Sensor configs
├── sensor_kit/autosdv_sensor_kit_launch/
├── vehicle/autosdv_vehicle_launch/
└── sensor_component/external/      # Sensor driver submodules
data/
├── COSS-map-planning/              # Default map
├── models/                         # ML models (YOLOX, CenterPoint)
└── zed-sdk/                        # ZED calibration
scripts/
└── leodrive-bus-launch/            # Leo Drive Bus-ODD dataset tools (submodule)
```

### Key Files
- Main launch: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
- Sensor calibration: `src/param/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`
- Preset files: `src/launcher/autosdv_launch/config/{perception,localization}/preset/`
- Web UI: http://localhost:8081 (via play_launch)

### 2-D MCL Localization (`pose_source:=mcl`)

Localizes a single-plane LaserScan against a 2-D occupancy grid instead of a PCD.
Measured on the Autoware sample site against NDT ground truth, five seeds:
**mean 0.789 m, p95 2.075 m, mean |yaw| 0.0159 rad** with a 3-ring scan source.

**The scan contract.** MCL consumes ONE `sensor_msgs/LaserScan` in any frame TF
connects to `base_link`, and holds no scan geometry of its own. The sensor kit
owns scan production, because which physical plane to use depends on the sensor
and its mounting:

```bash
# the kit publishes the scan (production)
just launch "pose_source:=mcl map_path:=data/my_site"
# MCL synthesises one from a 3-D cloud (test scaffolding only)
just sim logging "pose_source:=mcl scan_source:=test_pointcloud"
```

`mcl_scan_normalizer` resolves the mounting offset itself: `particle_filter`
treats the scan as originating at the particle pose, so a laser frame 0.5 m
forward of `base_link` would otherwise bias every range by 0.5 m.

**Kit-side scan production** from a 3-D LiDAR, in `autosdv_sensor_kit_launch`:

```bash
just launch publish_scan:=true      # plus scan_ring, or ring_min/ring_max
```

Use a **small ring group, not a single ring**. Measured comparison
(`docs/reports/2dlidar-scan-source-comparison.md`):

| scan source | mean | seed spread | gate | mean \|yaw\| |
|---|---|---|---|---|
| slab, 0.30 m band | 0.821 m | 0.072 | 5/5 | 0.0339 rad |
| 1 ring | 0.992 m | 0.317 | 3/5 | 0.0321 rad |
| **3 rings (70-72)** | **0.789 m** | **0.037** | **5/5** | **0.0159 rad** |

A single ring is geometrically a perfect plane but too sparse on a 128-ring
spinner. Three adjacent VLS128 channels span 0.22 deg (0.23 m at 60 m), which is
*tighter* than the slab they beat, so this is not a fidelity trade.

**Spinning LiDARs only.** Ring extraction assumes constant-elevation rings, so it
applies to `vlp32c` and not to the kit's solid-state sensors: Robin-W and Cube1
have restricted fields of view and a channel index that is not a fixed
elevation, so no ring is a horizontal plane. A narrow FOV also constrains MCL
poorly against a 360 deg grid. Use those sensors on the 3-D NDT path, or with a
native 2-D LiDAR alongside.

**The ring is sensor-specific and must be measured**, not copied -- 0.11 deg
channel spacing is a property of that VLS128:

```bash
python3 scripts/sensor/inspect_rings.py <bag> --topic <cloud> --height <mounting_h>
```

It reports per-channel elevation, names the horizontal ring, and warns when a
ring points too far up to meet the ground (a low vehicle can otherwise be
configured with a ring aimed at the sky).

**Map:** needs `occupancy_grid.yaml` + `.pgm` rather than a PCD; build with
`just map grid-from-pcd` or `map-grid-from-bag`, validate with `just map check`.

Design: `docs/design/mcl-user-setup-ux.md`. Diagnostics: the normaliser reports a
missing scan, a missing TF, an all-non-finite scan, and an out-of-plane mount --
every scan-side failure in this project's history was previously silent.

### CUDA Point Cloud Pipeline

Two whole-stage switches, both defaulting to `cpu`, plus the matcher:

```bash
just launch pointcloud_backend:=cuda \
            localization_pointcloud_backend:=cuda \
            pose_source:=cuda_ndt
```

| switch | selects |
|--------|---------|
| `pointcloud_backend` | sensing: crop-self, deskew, ring outlier filter |
| `localization_pointcloud_backend` | the NDT input chain: crop, voxel, random downsample |
| `pose_source` | the scan matcher |

**`pointcloud_backend:=cuda` works with `vlp32c` and `robin-w`, not `cube1`.**
Deskewing needs a per-point time offset. Nebula publishes `PointXYZIRCAEDT`,
which has one, and `seyond_ros_driver` does too from `autosdv-1.5.0` onwards —
with the default `POINT_TYPE`, since a driver built as `PointXYZIRC` publishes a
cloud the CUDA preprocessor rejects. The Blickfeld driver has no per-point time
at all, so `cube1` is refused with an error naming the reason.

**Everything must load into one container.** `cuda_blackboard` is not a
transport — it is a process-local map from an id to a device pointer, so a stage
in another process receives the id and finds nothing behind it.

The two filters Autoware does not ship (a standalone CUDA crop box, a CUDA
random downsample) live in `src/sensing/cuda_pointcloud_filters`. The package
skips itself when no CUDA toolkit is found.

**Fix those two filters upstream first.** They are
autowarefoundation/autoware_universe#13301, on
`jerry73204/autoware_universe:feat/cuda-standalone-filters`; that branch is
canonical, and this submodule mirrors it. The submodule goes away only once we
ship an Autoware release containing the merged PR — not by patching the Debian,
because `autoware-localrepo` builds official Autoware source and every
vehicle-side patch belongs here instead. Retirement steps:
`docs/design/cuda-pipeline-data-flow.md`.

Design and measurements: `docs/design/cuda-pipeline-data-flow.md`.

### CUDA NDT Localization

CUDA-accelerated NDT scan matching for faster localization on NVIDIA GPUs. This package is **maintained by AutoSDV** (not upstream Autoware).

**Package location:**
```
src/localization/cuda_ndt_matcher/  # AutoSDV-maintained, can be modified directly
├── cuda_ndt_matcher/               # Core CUDA NDT implementation
└── cuda_ndt_matcher_launch/        # Launch files and config
```

**Usage:**
```bash
# Use CUDA NDT instead of standard NDT
just launch pose_source:=cuda_ndt

# In logging simulation
just sim logging pose_source:=cuda_ndt
```

**Performance:** 1.3-1.6x faster than standard NDT, 57% less CPU usage on Jetson platforms.

**Development notes:**
- This package can be freely modified for AutoSDV-specific optimizations
- Uses same input/output interfaces as standard Autoware NDT
- Config files: `src/launcher/autosdv_launch/config/localization/ndt_scan_matcher/`

### Leo Drive Bus-ODD Dataset

The `scripts/leodrive-bus-launch` submodule provides tools for the [Leo Drive Bus-ODD dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) - an Autoware dataset with camera streams.

**Sensors in dataset:**
| Sensor | Model | Quantity |
|--------|-------|----------|
| LiDAR | Velodyne VLP16 | 1 (front) |
| LiDAR | Velodyne VLP32C | 2 (left, right) |
| Camera | Lucid Vision Triton 5.4MP | 3 |
| GNSS/INS | Applanix POS LV 120 | 1 |

**Usage:**
```bash
cd scripts/leodrive-bus-launch

# Full setup (download ~10.9GB + migrate to Autoware 1.5.0)
just setup

# Or step by step:
just setup-python    # Install rosbags Python package
just build           # Build ROS packages (applanix_msgs, sensor_kit, vehicle)
just download        # Download dataset
just migrate-all     # Migrate rosbags from autoware_auto_* to autoware_* msgs

# Play a migrated rosbag
just play data/all-sensors-bag1_migrated
```

**Packages included:**
- `leodrive_bus_sensor_kit_launch` - Sensor kit configuration
- `leodrive_bus_vehicle_launch` - Vehicle description (Isuzu bus)
- `applanix_msgs` - Applanix GNSS/INS message definitions

## Development

### Temporary Files
Write temp files to `./tmp/` (gitignored). Do NOT use system `/tmp/`.

### Build Notes
- With `--symlink-install`, edits to yaml/xml/py apply immediately (no rebuild needed)
- New files require rebuild to create symlinks
- First launch compiles TensorRT models (10-30 min) unless `just build-engines`
  was run first

### Autoware model data (`data_path`)

Autoware writes each compiled `.engine` next to the `.onnx` it built from. The
Debian package's tree at `/opt/autoware/1.5.0/data` is root-owned, so that write
fails, the engine is discarded, and the same models rebuild — and fail — on every
launch. `just setup-autoware-data` mirrors the tree into `data/autoware_data`
with symlinks (171 files, under a megabyte), and the top-level launch files
default `data_path` to it:

```yaml
default: "$(env AUTOSDV_DATA_PATH ./data/autoware_data)"
```

Override per launch with `data_path:=`, or globally with `AUTOSDV_DATA_PATH`.
Re-run after an Autoware upgrade — engines are tied to the TensorRT version and
the GPU, so they cannot be baked into an image built elsewhere.

**On amd64, the TensorRT version must match Autoware's to the patch.**
`autoware_tensorrt_common` compares an existing `.engine`'s recorded TensorRT
version against the macros baked into its own build and discards any
difference, so a workstation running a different patch of 10.x rebuilds all
five perception engines on *every* launch — nine minutes on an RTX 3090 — no
matter how many times `just build-engines` has run. `Depends: libnvinfer10`
does not express this; 10.9 and 10.16 satisfy it and both void the cache.
JetPack hides the problem on the vehicle, where the board's TensorRT *is* the
one the arm64 Debians were built against.

```bash
./setup.sh --run --only tensorrt-runtime --yes   # amd64: the matching runtime
just build-engines                               # warns if they still disagree
```

The required version is `nvidia_amd64.tensorrt_engine_abi` in `versions.yaml`;
`scripts/trt-runtime-env.sh` (sourced by `scripts/env.sh`) puts it ahead of the
host's own, and `scripts/version/trt-loaded-version.sh` prints what the loader
actually resolves — which is the only honest answer, since a workstation may
have several TensorRTs installed and `dpkg` names the wrong one.

### ROS 2 Launch Testing

**IMPORTANT**: When testing launch files, use `play_launch` instead of `ros2 launch`:

```bash
# PREFERRED: play_launch supports multi-stage kill (SIGINT → SIGTERM → SIGKILL)
play_launch launch autosdv_launch logging_simulation.launch.yaml

# If you must use ros2 launch directly, kill by process group (PGID) to avoid orphans:
ros2 launch autosdv_launch logging_simulation.launch.yaml &
LAUNCH_PID=$!
# ... do testing ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')  # Kill entire process group
```

**Killing play_launch by PGID** (for scripts/background processes):
```bash
# Start play_launch and capture PGID
setsid bash -c "play_launch launch autosdv_launch logging_simulation.launch.yaml" &
sleep 2
PLAY_PID=$(pgrep -f "play_launch.*logging_simulation" | head -1)
PGID=$(ps -o pgid= -p $PLAY_PID | tr -d " ")
echo $PGID > /tmp/sim_pgid.txt

# Later, kill by PGID (kills entire process group including child nodes)
kill -- -$(cat /tmp/sim_pgid.txt)

# Or directly if you have the PGID
kill -- -$PGID
```

**Why this matters**: Killing `ros2 launch` or `play_launch` with SIGKILL (`kill -9`) by PID only kills the parent process, leaving child processes (component_containers, nodes) as orphans. These orphan nodes continue running and consume resources. Always kill by PGID to terminate the entire process group.

**Cleaning up orphans** (if they occur):
```bash
# List orphan nodes
ros2 node list

# Kill all ROS-related processes
ps aux | grep -E "ros|component_container|autoware" | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

### Python Packages
Standard ROS 2 conventions: setup.py/setup.cfg, test files for copyright/flake8/pep257.

### Setup Script Architecture

`setup/` is a Python program with a `curses` menu, nothing to bootstrap (the
standard library only), and one list of steps:

```
setup/setup.sh          launcher; the repo root symlinks to it
setup/main.py           CLI
setup/autosdv_setup/registry.py   every step, in the order they must run
setup/autosdv_setup/model.py      Step, Requires, Machine (host detection)
setup/autosdv_setup/menu.py       the curses UI
setup/autosdv_setup/runner.py     execution
setup/autosdv_setup/state.py      what ran, what failed, what changed since
```

```bash
./setup.sh                              # pick a profile, then the steps
./setup.sh --status                     # what is installed
./setup.sh --list                       # every step, and whether it applies here
./setup.sh --run --profile vehicle --yes
./setup.sh --run --all --skip tensorrt-engines
./setup.sh --rerun opencv               # one step again
./setup.sh --run --dry-run --profile ci # print what would run, install nothing
./setup.sh --plain                      # numbered menu, when curses cannot drive the terminal
```

**Profiles**, not one flat list. Each step declares which of `dev`, `vehicle`
and `ci` selects it by default; `all` and `none` are computed, so a new step
joins them without being listed anywhere. `--profile vehicle --yes` is a
complete unattended install.

**Adding a step** is one entry in `registry.py`:

```python
Step(
    id="my-component",
    label="My Component",
    why="What it costs, and what breaks without it.",
    group="Libraries",
    run=[_S("install-my-component.sh")],   # argv, no shell
    requires=Requires(sudo=True, hardware="cuda"),
    after=("ros2",),                       # ordering only, never auto-selection
    profiles=_on(*DEV),
    verify=["bash", "-c", "command -v mything >/dev/null"],
)
```

If it is not in that list it does not run, and if it is in that list the UI
shows it. The previous design split this three ways — a `MENU_ITEMS` table in
`setup.sh`, the order in a justfile `setup:` chain, and a `_setup-*` wrapper per
option — so a step nobody wrote a wrapper for ran unconditionally and appeared
in no menu.

**Two mechanics worth knowing:**

- **Digests, not markers.** `Step.digest()` fingerprints the argv *and* the
  contents of any in-repo script it runs, so the UI can say "ran, but the script
  has changed since". A marker file cannot express that, and it is what bites
  when an install script is edited. State lives in `setup/.state.json`; the old
  `.markers/` directory is imported automatically on first run.
- **`--status` reads the machine where it can.** A step may carry a `verify`
  command, and where one exists its answer wins over the recorded run: a reboot
  drops the loopback `MULTICAST` flag, a JetPack OTA replaces the OpenCV
  headers, and an Autoware upgrade leaves the mirrored data tree pointing at a
  version that is gone.

`run` is argv executed without a shell. `_BASH(...)` and `_ros_bash(...)` are
the deliberate exceptions; the latter exists because ROS's setup hooks read
variables that are unset, so `set -u` has to come off around the sourcing.

### Preset System

AutoSDV uses a **preset system** (following Autoware's pattern) to manage component-level configurations. Presets group related parameters for common use cases.

#### How Presets Work

**Preset files** are YAML launch files that define launch arguments:

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
  # ... more args
```

**Main launch file** includes presets:

```yaml
# autosdv.launch.yaml
- arg:
    name: perception_preset
    default: "lidar_only"

- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

**Benefits**:
- ✅ Select presets for convenience: `perception_preset:=camera_lidar_fusion`
- ✅ Override individual parameters for experimentation: `use_traffic_light_recognition:=true`
- ✅ Easy to extend: Add new preset file without modifying launch files

#### Creating Custom Presets

1. Copy existing preset: `cp lidar_only_preset.yaml custom_preset.yaml`
2. Modify parameter defaults in the new file
3. Use with: `just launch perception_preset:=custom`

**Note**: Preset files must use `<name>_preset.yaml` naming convention.

#### Available Presets

**Perception** (`config/perception/preset/`):
- `lidar_only` - LiDAR only, no camera features (default)
- `camera_lidar_fusion` - Camera + LiDAR with traffic light recognition
- `minimal` - Minimal features for development/debugging

**Localization** (`config/localization/preset/`):
- `default` - Gyro odometry twist estimation (default)
- `eagleye` - GNSS-based odometry (requires GNSS)

See `config/{perception,localization}/preset/README.md` for detailed documentation.

## Quick Reference

### Common Launch Parameters

#### Preset-Based Configuration (Recommended)
```bash
# Perception presets (controls perception mode and features)
perception_preset:=lidar_only           # Default: LiDAR only, no camera features
perception_preset:=camera_lidar_fusion  # Camera + LiDAR with traffic light recognition
perception_preset:=minimal              # Minimal features for development

# Localization presets (controls twist estimation)
localization_preset:=default            # Default: gyro_odom
localization_preset:=eagleye            # GNSS-based odometry (requires GNSS)

# Example: Use camera-lidar fusion
just launch "perception_preset:=camera_lidar_fusion sensor_suite:=robin_zed"
```

#### Sensor Configuration
```bash
# Sensor suites (predefined combinations)
sensor_suite:=robin_zed          # Robin-W + ZED + ZED IMU
sensor_suite:=vlp32c_zed         # Velodyne + ZED + ZED IMU
sensor_suite:=vlp32c_zed_imu     # Velodyne + ZED + ZED IMU + MPU9250

# Individual sensor overrides
lidar_model:=robin-w|vlp32c|cube1
camera_model:=zedxm|usb|none
imu_source:=mpu9250|zed
gnss_receiver:=garmin|ublox|septentrio
```

#### Localization (pose_source)
```bash
# pose_source options:
pose_source:=cuda_ndt  # Default: CUDA-accelerated NDT (1.3-1.6x faster, 57% less CPU on Jetson)
pose_source:=ndt       # Autoware NDT (OpenMP CPU, fallback)
# pose_source_package defaults to "auto" and is derived from pose_source
# (cuda_ndt -> cuda_ndt_matcher_launch, otherwise built-in NDT). Set it
# explicitly only to plug in a third-party estimator. See
# docs/design/localization-method-switching.md.
pose_source:=mcl       # 2-D Monte-Carlo localization against an occupancy grid
```

#### System Features
```bash
# Localization
use_gnss:=false                  # Indoor operation (no GNSS)
use_ntrip:=true                  # RTK positioning (ublox only)
use_mapless_mode:=true           # Indoor operation without localization

# Perception
enable_zed_object_detection:=true  # ZED camera object detection
launch_perception:=false           # Disable entire perception module

# Advanced: Override preset-defined parameters
perception_mode:=lidar                      # Override preset perception mode
use_traffic_light_recognition:=true        # Override preset setting
use_detection_by_tracker:=false            # Override preset setting
use_image_segmentation_based_filter:=false # Override preset setting
use_pointcloud_map:=true                   # Override preset setting
twist_source:=gyro_odom|eagleye            # Override preset twist source
```

### Vehicle Interface (Quick Ref)

Values below are `actuator.yaml`'s, which is the file that drives the servo.
They drifted from it once; read them there when it matters.

**Motor PWM** (PCA9685 I2C, channel 0):
- `min_pwm` 360, `init_pwm` 370 (neutral), `max_pwm` 470, `brake_pwm` 340
- Multi-mode controller: Emergency Brake, Full Stop, Deadband Hold, Active Control (PID)

**Steering PWM** (PCA9685 I2C, channel 1):
- `min_steer` 439 (RIGHT, hardware is reversed), `init_steer` 489 (centre), `max_steer` 539 (LEFT)
- Steering limit, wheelbase and track are **not** in `actuator.yaml`. The
  actuator and `steering_status` read `max_steer_angle` (0.349 rad ≈ 20°),
  `wheel_base` (0.319) and `wheel_tread` (0.263) straight from
  `vehicle_info.param.yaml`, the file the planner sizes every trajectory
  against. They used to keep their own copy, and it drifted to 0.5 rad and
  0.340 m without anything noticing — do not reintroduce one.
- `tire_angle_to_steer_ratio` −1000 saturates the servo at **0.05 rad**, so in
  practice the steering limit never binds. Suspected ~7× too high; it needs a
  measurement on the vehicle, not an edit (`docs/known-config-defects.md` §8).
- Dual-mode controller: Fallback (v<0.3m/s), Normal (yaw rate feedback)

**Velocity Sensing**:
- Hall effect sensor (KY-003) on GPIO
- Parameters: `params/velocity_report.yaml`

**Actuator Parameters**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`

**Per-car values** live in Autoware's instance layer, `individual_params`
(`src/param/autoware_individual_params`), under `config/<vehicle_id>/`:
`autosdv_vehicle/actuator.param.yaml` (ESC neutral and brake points, servo
centre and ends, a measured steering ratio), `autosdv_sensor_kit/` (calibration,
IMU bias) and `autosdv.preset.yaml` (defaults for `sensor_model`,
`vehicle_model`, `sensor_suite`, `use_gnss`). Instances: `car_a` (VLP16 + ZED,
measured), `car_b` (Robin-W + ZED), `car_c` (a copy of B until measured), and
`sim`, the simulated nominal car (`actuator.yaml`'s values restated, Robin-W +
ZED preset; never a physical car). The launch argument `vehicle_id:=<car>` selects one, and it is
**required** on every launch that runs on a car (`just launch vehicle_id:=car_b`,
`just lab2 stack car_b`, `just control basic vehicle_id:=car_b`); there is no
default and no environment variable (`$(env VEHICLE_ID default)` is gone),
because the cars differ in their stop PWM (A 380, B 395) and the wrong one drives
instead of stopping. The preset is included before the arguments it defaults, so
the command line still wins (`vehicle_id:=car_a use_gnss:=true`). The actuator
file loads last into the actuator and gear manager. The simulators (`just sim
rc`, Lab 2 Part 1, the coach sim) pin `sim`, because their plant models
`actuator.yaml`; the vehicle benches (dry runs, no motor) default to `sim`;
the logging simulation defaults to `default` (it also carries
`sample_sensor_kit`). The lab2 `car:=A|B|C` config key maps to
`vehicle_id:=car_a|car_b|car_c` (never `sim`). Tools take `--vehicle-id`. A team adds its cars
in its own `individual_params`, never in ours. Design:
`docs/design/vehicle-variants-and-instances.md`.

## Documentation

### Main Documentation Book (MkDocs)

**The book is a SEPARATE REPOSITORY**, not a directory here and not a
submodule — `feb8a6f Remove book submodule` took it out of this tree. Editing
`book/` in this checkout edits nothing, because there is no `book/`.

- **Repository**: <https://github.com/NEWSLabNTU/AutoSDV-book> (`~/repos/AutoSDV-book`)
- **Published at**: <https://newslabntu.github.io/autosdv-book/>
- **Framework**: MkDocs with Material theme (following Autoware conventions)
- **Setup**: `just setup` (uv; installs dependencies)
- **Build**: `just build` (builds to `site/`)
- **Serve**: `just serve` (http://localhost:3000)
- **Check**: `just check` (dependency check, `mkdocs build --strict`, translation sync)
- **Source**: `src/` (Markdown files) — **`src/`, not `book/src/`**
- **Config**: `mkdocs.yml`

**Publishing is versioned, with mike.** A push to `main` deploys the `dev`
version; a release is an `X.Y-N` tag — `X.Y` the AutoSDV version, `N` the book
revision — which mike deploys and aliases to `latest`. The older `book-v*`
scheme is obsolete.

```bash
git tag -a 0.2-1 -m "Book revision 1 for AutoSDV 0.2"
git push origin 0.2-1
```

The live site serves `/latest/` (currently mike version `0.1`) and `/dev/`,
which tracks `main`.

Every English page has a `.zh-TW.md` sibling, so a new page is two files plus a
`nav` entry plus a `nav_translations` entry.

**Features**:
- ✅ Material Design theme
- ✅ Mermaid diagram support
- ✅ Multi-language (English + 繁體中文)
- ✅ Math rendering (MathJax)
- ✅ Search, dark mode, mobile responsive

**Key Guides** (in book, paths relative to that repository):
- **Sensor Integration**: `src/guides/sensor-integration/`
  - Simple usage guide, Robin-W walkthrough, sensor-specific details
- **Vehicle Control**: `src/guides/vehicle-control/`
  - Overview, hardware, control details, tuning & testing
  - Multi-mode controllers, PCA9685 I2C, hall effect sensor, PID tuning

**Revision in progress**: [docs/roadmap/8-book-revision.md](docs/roadmap/8-book-revision.md).
The published site is stale — it still says Autoware 2025.02 and gives `just`
recipes that no longer exist.

### Legacy Guides (docs/)
| Guide | Description |
|-------|-------------|
| [docs/guides/sensor_configuration.md](docs/guides/sensor_configuration.md) | Sensor suites, NTRIP/RTK, localization |
| [docs/guides/ndt-tuning.md](docs/guides/ndt-tuning.md) | NDT localization tuning: process, pitfalls, checklist |
| [docs/reports/localization-open-questions.md](docs/reports/localization-open-questions.md) | Known-but-unfixed localization items, with how to settle each |
| [docs/known-config-defects.md](docs/known-config-defects.md) | Configuration that is wrong on every run until someone edits a file |
| [docs/guides/vehicle_calibration.md](docs/guides/vehicle_calibration.md) | PWM control, PID tuning, testing tools |
| [docs/guides/zed_camera.md](docs/guides/zed_camera.md) | ZED setup, troubleshooting |
| [docs/guides/lidar_integration.md](docs/guides/lidar_integration.md) | Robin-W, Velodyne, TensorRT |
| [docs/guides/control_testing.md](docs/guides/control_testing.md) | Control system testing procedures |
| [docs/guides/mrm_configuration.md](docs/guides/mrm_configuration.md) | MRM (emergency stop) configuration |
| [docs/research/localization/ndt_parameter_tuning_coss_map.md](docs/research/localization/ndt_parameter_tuning_coss_map.md) | NDT tuning research |

## Known Issues

- **Steering reversed**: Left/right inverted in manual control
- **No steering feedback**: the vehicle has no steering angle sensor, so
  `/vehicle/status/steering_status` republishes the *command* (see
  `docs/reports/steering-status-has-no-feedback.md`). MPC consumes it as its
  controller state, so the lateral loop is closed on its own output. Reads 0
  in any bag recorded while disengaged.
- **Network monitor errors**: AWS Greengrass socket errors (non-critical, ignore)
- **ZED in VNC**: Requires TurboVNC with VirtualGL for hardware acceleration
## Important Notes

- **Autoware 1.5.0**: Installed at `/opt/autoware/1.5.0/` via the setup script (autoware-localrepo)
- **The installed Autoware is official Autoware.** `NEWSLabNTU/autoware-localrepo`
  builds the Debians from official source, so `/opt/autoware/<version>` is a
  clean baseline: what it does is what upstream does, and a bug reproduced
  against it is reportable. Every patch of ours lives in this repository or one
  of its submodules instead — including patches to packages Autoware owns, which
  are carried as a submodule under `src/` until they land upstream. Do not fix
  an Autoware bug by patching the localrepo build; the cost is an installed
  Autoware that silently is not Autoware, and nobody reading a launch file can
  see it.
- Source Autoware environment: `source /opt/autoware/1.5.0/setup.bash` (includes ROS 2)
- Source ROS only: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble, Ubuntu, NVIDIA GPU
- Uses colcon (not catkin)
- Logs: `play_log/latest/`
- Stop system: Ctrl+C
