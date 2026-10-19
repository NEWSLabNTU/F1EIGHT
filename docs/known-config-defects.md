# Known configuration defects

Things that are wrong in configuration rather than in hardware or code, and that
therefore stay wrong on every run until somebody edits a file. Separate from a
bug list: none of these is blocking anything, which is exactly why they survive.

Seeded on 2026-09-11 from the golf cart's ledger of the same name, checking each
of its entries against this repository. That vehicle runs the same Autoware
1.5.0, so its findings are a good prior — but every entry below was verified
here rather than copied, and several did not transfer.

**How the entries were checked.** Statically, against the files and the
installed Autoware, because no stack was running. Anything that needs a live
diagnostic graph is listed under *Needs a run* rather than asserted.

**Read a config out of git, not out of a submodule working tree.** A working
tree can be ahead of, behind, or unrelated to what the recorded pin builds, so a
conclusion drawn from it may be about code nobody else has. The one-line guard:

```bash
git submodule status --recursive | grep '^+'
```

---

## 1. RViz asked for an MRM overlay plugin that does not exist — FIXED 2026-09-11

`autosdv.rviz` and `mapless.rviz` both carried:

```yaml
- Class: rviz_plugins/MrmSummaryOverlayDisplay
  Enabled: false
```

Autoware 1.5.0 declares 21 `rviz_plugins/*` classes across its
`plugin_description.xml` files, and that is not one of them. Every one of the
other classes our configs name **is** there, so this was the single dead
reference, checked by resolving all of them:

```bash
grep -rho 'name="rviz_plugins/[A-Za-z]*"' /opt/autoware/1.5.0/share/*/plugins/plugin_description.xml
```

Consequence while it stood: RViz logged a `PluginlibFactory` failure at every
startup and carried on without the display. Since it was `Enabled: false` it
never rendered anything, so nothing was lost by removing it — but the conclusion
it supports stands: **RViz has never shown MRM state on this machine.** Anything
that assumed that coverage exists was assuming wrong.

The install does ship `autoware_overlay_rviz_plugin/SignalDisplay` and
`autoware_string_stamped_rviz_plugin/StringStampedOverlayDisplay` if an overlay
is wanted later.

---

## 2. `topic_state_monitor_initialpose3d` has all-zero thresholds

`config/system/component_state_monitor/topics.yaml`, and the `topics-mcl.yaml`
variant beside it:

```yaml
node_name_suffix: initialpose3d
warn_rate: 0.0
error_rate: 0.0
timeout: 0.0
```

A monitor with a zero timeout and a zero error rate cannot express a healthy
state. It is a graph leaf that is red by construction, which is worse than an
absent one: absent is visible, permanently red is background.

**Not our doing.** Autoware 1.5.0's own
`autoware_launch/config/system/component_state_monitor/topics.yaml` carries the
same three zeros, byte for byte. Whatever is right here is right upstream too,
which is an argument for reporting it there rather than diverging quietly.

The golf cart measured this one live: ERROR on 100% of 3,219 reports.

---

## 3. Autoware's `system_monitor` runs on stock defaults, which assume a desktop — FIXED 2026-10-19 (needs a relaunch to confirm)

We do ship `config/system/system_monitor/*.param.yaml`, but
`components/autosdv_system_component.launch.xml` passes
`$(find-pkg-share autoware_launch)/config/...` for every one of them, so **our
copies are not what runs.** The stock parameters assume every network interface
is up, an `hdd_reader` daemon, an NVML GPU and a CMOS battery. On a Jetson:

| Monitor | Why it cannot pass |
|---|---|
| `net_monitor` | `devices: ["*"]`, and `l4tbr0` is always down on a Jetson. One down interface makes the whole check ERROR forever. |
| `hdd_monitor` | `hdd_reader_socket_path: /tmp/hdd_reader.sock`; no such daemon runs here. |
| `gpu_monitor` | Tegra is not NVML. |
| `voltage_monitor` | `cmos_battery_label: ""`, and the Orin has no CMOS battery. |

Our own `net_monitor.param.yaml` also still says `devices: ["*"]`, so pointing
the launch at it would fix nothing until the interfaces are named.

The golf cart fixed this by naming the interfaces and not launching the three
monitors a parameter cannot fix, which needed a local copy of
`tier4_system_component.launch.xml`, because `tier4_system_launch` hardcodes its
include of `autoware_system_monitor` and takes no per-monitor argument.

Measured on cars A, B and C (AGX Orin, 2026-10-19), which matched the table, plus
two more: `cpu_monitor`'s "CPU Thermal Throttling" is ERROR "No message was set"
(the installed `arm_cpu_monitor` build does not implement it), and the
`voltage_monitor` reading is the Tegra RTC printing no `batt_status` line, not
the coin cell. The installed `gpu_monitor` is the `unknown_gpu_monitor` build.

**Fix.** `autosdv_system_component.launch.xml` passes `launch_system_monitor:=false`
to `tier4_system_launch` and runs `components/jetson_system_monitor.launch.xml`
instead (same namespace, container and node names), with our parameter files,
refreshed from 1.5.0. It starts cpu, mem, net, ntp and process monitors;
`launch_hdd_monitor`, `launch_gpu_monitor` and `launch_voltage_monitor` default
to false. `net_monitor` watches `lo` and `eno1` only. Left: "CPU Thermal
Throttling" stays ERROR (no parameter turns it off). None of these items is in
the diagnostic graph, so none ever gated autonomous mode.

---

## 4. The web monitor listed `/diagnostics_agg` — FIXED 2026-09-11

Nothing publishes it. Autoware uses `autoware_diagnostic_graph_aggregator`,
which publishes `DiagGraphStruct` and `DiagGraphStatus` rather than an
aggregated `DiagnosticArray`, so the row was permanently dead in the monitor's
own table. Removed with the phase 7 monitor fixes.

Its three GNSS siblings were worse than dead: `nmea_msgs/msg/Sentence`,
`rtcm_msgs/msg/Message` and `ublox_msgs/msg/RxmRTCM` were named in the config
but missing from the node's type map, so the loader skipped them and the table
showed NO DATA whether or not anything published. Also fixed.

---

## 5. The DDS environment depends on how you enter the workspace — FIXED 2026-09-13

Found on 2026-09-12, while trying to measure the sensing chain.

`install/setup.bash` does **not** set `RMW_IMPLEMENTATION`. A script that sources
only the workspace overlay therefore runs on Fast-DDS, while this repo's
kernel-buffer setup step, its `cyclonedds.xml`, and CLAUDE.md all assume
CycloneDDS.

Nothing errors. Discovery half-works, and the symptoms look like broken code
rather than a broken transport:

- `ros2 topic list` returns two topics while the stack is running 130 members.
- `ros2 topic echo` reports a topic as unpublished while `ros2 topic hz` on the
  same topic is printing rates — echo resolves the type through the graph
  inventory, a direct subscriber does not.
- The same measurement reads 3.4, 9.1, 11.3 or 20.2 Hz on different runs.

Sourcing `/opt/autoware/1.5.0/setup.bash` before the overlay fixes it: that is
where `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` comes from.

Two related facts:

- **`.envrc` sets the RMW only in its fallback branch.** The `if` branch sources
  Autoware and returns; the `export RMW_IMPLEMENTATION` and `export
  CYCLONEDDS_URI` lines live in the `else`, which runs only when Autoware is
  *absent*. On any machine with Autoware installed they never execute. It works
  anyway, because Autoware's setup.bash sets both — but not because this repo
  does.
- **The repo's own `cyclonedds.xml` is not what runs.** Autoware exports
  `CYCLONEDDS_URI=file:///opt/autoware/1.5.0/config/cyclonedds.xml`, and nothing
  overrides it. Whatever tuning the repo's copy carries has never been in
  effect on a machine with Autoware installed.

**Also**: stale `/dev/shm/fastrtps_*` segments accumulate from killed runs and
break Fast-DDS shared memory outright ("open_and_lock_file failed"). 473 had
collected here. `rm -f /dev/shm/fastrtps_*` when a graph starts behaving oddly.

### The fix, 2026-09-13

`scripts/env.sh` sources the Autoware prefix from `versions.yaml` and then the
workspace, and every runtime entry point uses it: `just/{sim,tool,control}.just`,
`demo/justfile`, `demo/scripts/run-coss-ndt.sh`, the root justfile's `launch`.
`just build` and `just test` still source ROS alone, deliberately — the
middleware does not matter to a compiler.

What it cost before the fix, on a clean shell:

- `just demo run` with `pose_source:=ndt` left the pose seeder waiting out its
  whole 60-second timeout — "no /clock after 60 s -- is the bag playing with
  --clock?" — while the bag had been playing the entire time.
- The same run recorded a 24 KiB bag holding **zero** messages, while
  `play_launch` reported every node ready.
- 197 stale `/dev/shm/fastrtps_*` segments had collected from earlier runs that
  failed the same silent way.

The two related facts above are unchanged: `.envrc` still sets the RMW only in
its fallback branch, and the repo's own `cyclonedds.xml` still never runs.

---

## 6. The Autoware 1.5.0 Debian exports paths from the machine that built it

Four packages ship CMake target exports that name the build container's
workspace instead of the install prefix:

```text
CMake Error in CMakeLists.txt:
  Imported target
  "autoware_pointcloud_preprocessor::pointcloud_preprocessor_filter_base"
  includes non-existent path

    "/output/workspace/install/autoware_point_types/include"

  in its INTERFACE_INCLUDE_DIRECTORIES.
```

`/output/workspace` exists only inside the Debian build container. The headers
are really at `/opt/autoware/1.5.0/include`. Affected exports:

| File | Package |
|---|---|
| `share/autoware_pointcloud_preprocessor/cmake/export_autoware_pointcloud_preprocessorExport.cmake` | `autoware_pointcloud_preprocessor` |
| `share/nebula_decoders/cmake/export_nebula_decoders_{robosense,robosense_info,continental}Export.cmake` | `nebula_decoders` |
| `share/nebula_ros/cmake/export_{hesai,robosense,velodyne}_ros_wrapperExport.cmake` | `nebula_ros` |
| `share/llh_converter/cmake/ament_cmake_export_include_directories-extras.cmake` | `llh_converter` |

**What it breaks.** Nothing this repository builds today — no package here links
those imported targets. It bites the moment you build a package against them
out of tree, which is how it was found: a colcon build of a patched
`autoware_cuda_pointcloud_preprocessor` failed at *CMake Generate step failed*
before compiling a single file.

**Where the fix belongs.** In `NEWSLabNTU/autoware-localrepo`, not here: the
packaging step should rewrite `/output/workspace/install/<pkg>/include` to the
install prefix, the way ament's own export generation does for in-tree
dependencies. Until then, a local build against those targets needs the path to
exist — `sudo mkdir -p /output/workspace/install/autoware_point_types/include`
is enough to get past generate, because CMake only checks existence.

---

## 7. The vehicle interface kept its own copy of the vehicle's geometry — FIXED 2026-10-05

`actuator.yaml` declared `wheelbase: 0.340`, `track_width: 0.24` and
`max_steering_angle: 0.5`, while `vehicle_info.param.yaml` — the file the
planner and MPC size every trajectory against — says `wheel_base: 0.319`,
`wheel_tread: 0.263` and `max_steer_angle: 0.349`. Nobody had measured the
actuator's values: they were a hard-coded dataclass default that
`autosdv_vehicle_launch` `4cbd2f6` moved into YAML unchanged. This file's own
quick reference had been saying 0.349 for months while the YAML said 0.5.

Fixed by deleting the copy rather than correcting it (`autosdv_vehicle_launch`
`046230a`). `actuator` and `steering_status` now declare Autoware's names and
the launch file loads `vehicle_info.param.yaml` into them, overridable as
`vehicle_info_param_file:=`. Verified by launching the interface and reading
the parameters back: `wheel_base` 0.319, `wheel_tread` 0.263,
`max_steer_angle` 0.349, and the old names unset.

This changed effectively nothing on the vehicle, and §8 is why.

---

## 8. `tire_angle_to_steer_ratio` saturates the servo at 0.05 rad

```python
steer_pwm = init_steer + int(average_angle * tire_angle_to_steer_ratio)  # ratio -1000
steer_pwm = max(min_steer, min(max_steer, steer_pwm))                    # 439..539
```

The servo window is ±50 counts around 489, so at −1000 counts/rad the clamp is
reached at **|δ| = 0.05 rad, 2.9°**. Every command beyond that is full lock.
Both steering limits in §7 are far outside this, which is why they never
mattered.

If ±50 counts really is the ±20° that the book and the quick reference in
`CLAUDE.md` claim, the ratio should be about 50 / 0.349 ≈ **143** counts/rad,
and the controller is commanding roughly **seven times** the steering it
intends below 2.9°. Nothing on the vehicle would report that: there is no
steering-angle sensor, and `steering_status` republishes the command
(`docs/reports/steering-status-has-no-feedback.md`), so MPC sees exactly the
angle it asked for.

**Not fixed, because it needs a measurement, not an edit.** The ±20° claim is
undocumented too, and changing the ratio on its strength would swap one
unmeasured number for another. To settle it, on the vehicle:

1. Hold `steer_pwm` at fixed offsets from 489 — say ±10, ±25, ±50 — and drive
   slow full circles at each.
2. Measure the circle radius `R` and convert: `δ = atan(wheel_base / R)` with
   `wheel_base` 0.319.
3. The ratio is `offset / δ` in counts per radian, so fit offset against δ (the
   slope of δ against offset is its inverse). The sign comes out of the fit.
   The δ at ±50 is the true `max_steer_angle`, which belongs in
   `vehicle_info.param.yaml`.

**The procedure is now a tool:** `just lab2 steer-hold OFFSET` holds the PWM,
`just lab2 steer-fit OFFSET:RADIUS ...` fits it, and
`docs/guides/lab2-field-calibration.md` ("Car gate: steering ratio and lock")
says where the values go. This defect stays open until a car has been
measured; Lab 2 loads a -143 stand-in meanwhile.

---

## Did not transfer

Checked, and not defects here:

- **Two nodes claiming `/sensing/gnss/ublox`.** One `ublox_gps` node, in
  `gnss.launch.xml`, guarded by `launch_driver`.
- **`aruco_planning_sim.launch.xml` missing the simulator component's required
  arguments.** That launch file is golf-cart-only.
- **A wrong comment in the sensor kit's analyzer config.** Ours
  (`config/diagnostic_aggregator/sensor_kit.param.yaml`) carries no such claim.

## Needs a run

These cannot be settled by reading files. Each needs the stack up, and the
golf cart's observation is the prior, not the finding:

- `collision_detector` reports ERROR with no message.
- `/adapi/node/vehicle_door` reports a door status forever on a vehicle with no
  door sensing.
- The overall share of diagnostic reports that are ERROR or STALE. On the golf
  cart it was **31.1%**, nearly all of it configuration. A graph that is
  permanently a third red trains everyone to ignore it, which is the reason this
  document exists.

Run `scripts/testing/localization/` and the web monitor together on the board,
and add what they show.
