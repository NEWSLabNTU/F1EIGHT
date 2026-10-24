# Lab 3: indoor map building and NDT localization

The AutoSDV side of Lab 3: a team drives a car around the classroom, builds a
point-cloud map with GLIM, moves it into the reflective board's frame, draws a
clockwise lanelet2 loop in Vector Map Builder, and in week 2 drives that loop
autonomously on NDT, started from the board. Package `src/lab/autosdv_lab3`,
recipes `just lab3` (`just/lab3.just`). The lab's plan and the design behind this
package are in the course repository, `2026/labs/Lab3/`.

## The path

```
week 1, hall     just lab3 stack car_b                       terminal 1
                 just lab3 record --car car_b --team 3       terminal 2; checked on Ctrl-C
week 1, Orin     just lab3 map BAG                           GLIM -> BAG_glim/raw_map.pcd
anywhere         just lab3 anchor BAG_glim/raw_map.pcd MAP --car car_b
                 just lab3 downsample MAP --voxel 0.1
                 (Vector Map Builder -> MAP/lanelet2_map.osm)
                 just lab3 check-lanelet MAP                 requirements + simulator lap
week 2, hall     just lab3 record --car car_b --team 3 --kind run --map MAP   terminal 2, FIRST
                 just lab3 run car_b MAP                     terminal 1
                 just lab3 route MAP --engage --laps 2
TA               just lab3 grade --map MAP --run RUN_BAG --checkpoints CP_BAG \
                     --marks marks.yaml --sheet sheet.yaml [--fallback-map DIR]
```

The run bag is started before the stack: the board pose that initializes it is not
latched, so a bag that begins after INITIALIZED cannot show the board did it.

## Week-2 grading (`lab3_grade`)

The marks of the course plan's week-2 table, from the bags, on fixed lines (no TA
reference run). The module docstring of `autosdv_lab3/grade.py` holds every line.

| item | marks | from |
|---|---|---|
| board initialization | 5 | INITIALIZED <= 30 s after `/lab3/run_meta`'s launch time; no `/initialpose`; the first NDT pose within 0.5 m and 10 deg of the board's |
| checkpoint accuracy | 15 | each mark's first stop of >= 3 s within 1 m: half position (0.10 m full, 0.40 m zero), half heading (3 deg full, 15 deg zero) |
| autonomous laps | 15 | the first four legs `/lab3/route_events` announces, 3.75 each: arrived within 2.5 m of the goal, autonomous >= 90 % of the leg, no route set by hand, no takeover on the TA's sheet |
| staying localized | 10 | over the graded legs, 2.5 each: no re-initialization; no NDT pose jump > 0.3 m beyond what the speed explains; NVTL >= the indoor gate >= 95 % of the time; median scan-to-map residual <= 0.10 m |
| fallback map | -10 | `/lab3/run_meta`'s map hash is a `--fallback-map` folder's |

What the stack adds to the bag for this: `lab3_run_meta` (in `lab3_run.launch.xml`)
latches the launch time and the loaded map's hash; `lab3_route` announces each leg it
sets and each arrival. The transmitter's manual switch is hardware and leaves no trace,
so the TA's sheet lists those takeovers. An item the bag cannot decide (no run meta, a
map hash that is neither map, a recorder started late) reads `—` and the total is left
to the TA.

The residual places each sampled scan (one a second, 3000 points) by the localization
pose, `base_link` on the map's floor (z 0), and measures the nearest map point, capped
at 1 m; a coarse voxel raises it, which is the trade the team chose.

## The map folder

| file | written by | what |
|---|---|---|
| `anchored_full.pcd` | anchor | the GLIM cloud in the board frame, full resolution; not handed in |
| `pointcloud_map.pcd` | downsample | what NDT and Vector Map Builder load |
| `board_anchor.yaml` | anchor | T(map <- GLIM) and the board detection |
| `board_profile.yaml` | anchor | the detector profile used; the week-2 initializer loads this copy |
| `map_projector_info.yaml` | anchor | `projector_type: Local` |
| `lanelet2_map.osm` | the team | the clockwise loop |
| `lab3_map.yaml` | every step | provenance: inputs, hashes, settings, AutoSDV commit |

The board frame: origin on the floor below the board centre, x out of the wall along
the board normal, z up. Every team's map shares it.

**Anchor before downsampling.** The board is 0.6 m wide; a coarse voxel grid leaves
it too few points to find. `downsample` refuses a folder without an anchor, and
always starts from `anchored_full.pcd`.

## Per car

| car | LiDAR | raw topic | frame | board profile |
|---|---|---|---|---|
| A | VLP-16 | `/sensing/lidar/velodyne_points` | `vlp16` | `config/board/lab3_vlp16.yaml` |
| B, C | Robin-W | `/sensing/lidar/iv_points` | `robin_w` | `config/board/lab3_robin_w.yaml` |

These are what the drivers publish (`pointcloud_preprocessor.launch.py`,
`LIDAR_TOPICS`); the launch-argument defaults that name other topics are overridden.
`autosdv_lab3/cars.py` holds the table.

## Provisional values

None of these has been measured on a car; each says so in its file.

- **Board profiles** (`config/board/lab3_*.yaml`): a board mounted low, centre 0.4 m
  (bottom edge 0.1 m), so the beams meet it from about 1 m out; at the golf cart's
  1.3 m the VLP-16 (+15 deg up) would need 4-5 m. Intensity threshold 100 (the
  Velodyne convention), the sensor height 0.30 m standing in for an unmeasured
  extrinsic. Measure `pose_in_map`'s height on the mounted board.
- **Indoor NDT** (`config/ndt_indoor.param.yaml`): resolution 1.0 (outdoor 2.0) and
  the NVTL gate 1.3 (outdoor 2.0), moved together because NVTL scales with the voxel
  size. Built-in NDT only (`pose_source:=ndt`, the `run` default): the CUDA matcher
  reads its own file. `ndt_param_file` is a new `autosdv.launch.yaml` argument.
- **Size limit** 50 MB (`downsample --max-mb`), until Vector Map Builder's upload
  limit is measured.
- **Bag checks** (`check-bag`): 60 s, 0.8 m/s, 100 Hz IMU, 8 s still at each end, the
  board detected in the first and last 15 s.

## Known limits

- **The board needs driving past.** GLIM adds a keyframe every 0.15 m of travel, so a
  car parked in front of the board contributes one scan to the map. Pass it slowly on
  every loop.
- **Autoware sets the corner radius, not the car.** The planning validator rejects
  curves tighter than 1 m; in the RC simulator 1.2 m corners stalled and 1.6 m ones
  drove. `check-lanelet` requires 1.5 m (`PLANNER_MIN_RADIUS`, provisional).
- **A lap is two legs.** The mission planner refuses a route that enters a lanelet
  twice, so `route.py` drives a lap as two routes with a stop between. Every lap reuses
  the first lap's goals: re-planning from where the car stopped crept the goals 2.5 m
  back per lap, into the corners.
- **Planning runs with `planning_profile:=rc`** (`lab3_run.launch.xml`, the simulator
  lap). Autoware's stock planning left the car parked at the start of lap 2: from a
  stop about 10 deg off the lane the start planner either planned a back-out the car
  cannot drive or, with reversing off, found no forward path and held it. The profile
  drops the start planner and the reversing manoeuvres, halves the path interval and
  allows the car's own curvature; `autosdv_launch/config/planning_rc/README.md` lists
  each change. Measured in the RC simulator: 5 of 5 runs of three laps (15 laps, 30
  route changes). Before the goal planner's reversing modes were off, one run in three
  lost the behavior planning container to an rclcpp crash on a route change; not seen
  since, but not ruled out.
- **A counter-clockwise loop routes but does not drive** in the simulator (validator
  curvature), even where its clockwise mirror does. The check fails it on requirement
  3 regardless.
- **This workstation's DDS.** `cyclonedds.xml` asks for a 10 MB socket buffer; with
  `net.core.rmem_max` at 1 MB no node starts. Raise it (`sudo sysctl -w
  net.core.rmem_max=10485760`) or point `CYCLONEDDS_URI` at a copy with
  `min="default"`. Large clouds (a Robin-W scan is about 4 MB) still drop at 1 MB.

## On a car, still to verify

The cars' LiDAR extrinsics (`sensor_kit_calibration.yaml` is zeros), the topics on a
running stack, GLIM's run time on the Orin, the board profiles on a real scan, the
indoor NDT, the initializer's first align, and Autoware's planning and control
driving the loop on a real car.
