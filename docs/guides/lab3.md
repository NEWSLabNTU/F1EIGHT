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
week 2, hall     just lab3 run car_b MAP                     terminal 1
                 just lab3 record --car car_b --team 3 --kind run --map MAP
                 just lab3 route MAP --engage --laps 2
```

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
  twice, so `route.py` drives a lap as two routes with a stop between.
- **The second lap stalls in the simulator.** The car stops lap 1 about 13 deg off the
  lane, and Autoware's start planner then plans a back-out; the car cannot reverse.
  Moving goals onto straights did not fix it. Both the car and the RC simulator plan
  with `autoware_launch`'s stock planning files (`autosdv_launch/config/planning` is
  an unused, pre-1.5.0 copy), so the fix is a planning override for the RC scale
  (start planner `enable_back` off, path interval, validator curvature), not yet made.
  One lap per `just lab3 route` call works.
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
