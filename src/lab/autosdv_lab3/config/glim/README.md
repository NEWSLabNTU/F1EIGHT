# GLIM config templates for `just lab3 map`

`lab3_map` writes a complete GLIM config directory per run (`<out>/glim_config/`), in
four layers, later wins, key by key:

1. GLIM's installed defaults, `/opt/ros/humble/share/glim/config/*.json`
2. `common/` -- the classroom tuning, every sensor
3. `vlp16/` or `robin_w/` -- the sensor family, picked from the LiDAR topic
   (`/sensing/lidar/velodyne_points` -> vlp16, `/sensing/lidar/iv_points` -> robin_w)
4. what only the bag knows: the LiDAR and IMU topics, `T_lidar_imu` from the bag's own
   `/tf_static`, the log directory, and `config.json` choosing the GPU or `--cpu` modules

Files here hold only the keys they change, as plain JSON. The generated directory is
what ran, so read that, not this, when a map looks wrong.

The tuning is for a classroom, about 10 x 15 m, driven at <= 0.8 m/s. GLIM's defaults
are for outdoor mapping over hundreds of metres.

## common/

| file | key | value (default) | why |
|---|---|---|---|
| config_ros | `extension_modules` | `[]` (viewer, rviz, memory monitor) | headless: no window on the car or on anyone's display |
| config_odometry_gpu | `voxel_resolution`, `_max` | 0.2, 0.5 (0.25, 0.5) | walls and desks are metres away, not tens |
| | `voxel_resolution_dmin`, `_dmax` | 2, 8 m (5, 20) | the range band over which voxels coarsen, scaled to the room |
| | `num_threads` | 4 (2) | the Orin has the cores |
| config_odometry_cpu | `ivox_resolution`, `vgicp_resolution` | 0.5, 0.25 (1.0, 0.5) | as above, for `--cpu` |
| config_sub_mapping_{gpu,cpu} | `keyframe_update_strategy` | DISPLACEMENT (OVERLAP) | in a closed room every scan overlaps the last keyframe by more than the 0.6 threshold, so overlap-based keyframing almost never fires and a submap would span minutes of odometry drift. Displacement makes one every 0.15 m / 0.2 rad |
| | `keyframe_update_interval_trans`, `_rot` | 0.15 m, 0.2 rad (1.0, 3.14) | only keyframes reach the map, so their spacing is the map's density: a VLP-16 ring hits the board at a different height from each position, and every 0.15 m at 0.8 m/s is about every second scan |
| | `max_num_keyframes` | 40 (15) | so a submap still covers about 6 m of driving and the global graph stays small |
| | `submap_downsample_resolution` | **0.03 m** (0.1 GPU, 0.3 CPU) | the exported map's point spacing; see below |
| | `submap_target_num_points` | -1, no cap (50000 GPU) | a cap randomly thins a submap after the voxel step, which would thin the board too |
| | `keyframe_voxel_resolution` | 0.15 (0.25) | registration voxels inside a submap |
| | `submap_voxel_resolution` | 0.25 (0.5) | the submap voxel map the global stage registers against |
| config_global_mapping_{gpu,cpu} | `max_implicit_loop_distance` | **15 m** (100 m) | candidate pairs for submap-to-submap factors; the room's diagonal is ~18 m and the drivable loop smaller, so every overlapping pair qualifies and nothing outside the room does |
| | `submap_voxel_resolution`, `_max`, `_dmin`, `_dmax` | 0.25, 0.5, 2, 8 (0.5, 1.0, 5, 20) | as for odometry |
| | `randomsampling_rate` | 0.15 GPU, 0.05 CPU (1.0, 0.2) | submaps are now ~10x denser (3 cm, no cap), so factors sample fewer of their points to keep the same cost |
| config_global_mapping_pose_graph | `min_travel_dist` | **5 m** (50 m) | not selected by default; if someone switches to the pose-graph back end, its 50 m default gives a classroom loop no loop candidates at all (the golf cart's lesson) |
| | `max_neighbor_dist`, `vgicp_voxel_resolution`, `gicp_max_correspondence_dist` | 3, 0.5, 1.0 (5, 2.0, 2.0) | scaled to the room |

## vlp16/, robin_w/

| file | key | value (default) | why |
|---|---|---|---|
| config_preprocess | `num_threads` | 4 (2) | |
| config_sensors | `intensity_field`, `autoconf_perpoint_times` | `intensity`, true | both drivers publish Autoware's PointXYZIRCAEDT: uint8 `intensity` and uint32 `time_stamp` (ns from the scan start), which GLIM recognises and scales itself. Stated rather than inherited so a GLIM default change cannot drop the intensities |

The two families currently tune the same; the directories are where a sensor-specific
change goes (the Robin-W's narrow field of view is the likely first one, once a Robin-W
mapping bag exists).

Preprocessing is deliberately left at GLIM's defaults (1.0 m random-grid cell, 10000
points per scan, 0.5-100 m). Each of the obvious classroom changes was tried on the
outdoor bag and made odometry worse, measured against stock GLIM (17.4 m net travel,
parked z jitter 1 mm):

| change | net travel | parked z jitter |
|---|---|---|
| `distance_far_thresh` 30 m | 5.8 m | 0 |
| `downsample_resolution` 0.25 + target 20000 | 17.4 m | 91 mm |
| `random_downsample_target` 20000 | 17.2 m | 10 mm |

The map's density comes from the submap merge below, not from more points per scan.

## Point density and intensity: what reaches raw_map.pcd

The board anchor (F4) has to find a 0.6 x 0.6 m retroreflective board, intensity above
~100, in `raw_map.pcd`, so the export keeps the board densely sampled:

- **Per scan**, preprocessing keeps 10000 points by voxel-random sampling
  (`gtsam_points::randomgrid_sampling`, about the same count in every 1 m cell, so the
  board is not starved by nearer surfaces): real points, not averages, so each keeps
  its own intensity.
- **Per submap**, GLIM merges the keyframes' deskewed scans with
  `gtsam_points::merge_frames(..., submap_downsample_resolution, submap_target_num_points)`,
  which keeps ONE ORIGINAL POINT PER VOXEL, chosen by reservoir sampling, with all its
  attributes -- intensity is copied, never averaged. At 0.03 m that is at most ~400
  points on the board per submap, and a bright return is not diluted by a dark
  neighbour.
- **Across submaps**, the exporter concatenates every submap (T_world_origin * p) and
  does not downsample again; the board appears once per submap that saw it, so the
  map's spacing on surfaces seen often is finer than 3 cm. `just lab3 downsample` is
  where the size comes down.

Measured on `data/rosbags/outdoor_20251226_153115` (VLP-32C, outdoors, parked 116 s
then ~18 m of driving): 974k points in 3 submaps; median nearest-neighbour spacing
4.9 cm for points within 5 m of the trajectory, 5.3 cm within 10 m (p90 10 cm), against
7.0 cm overall (9.1 cm with keyframes every 0.4 m). Intensities come through as the driver's
0-255 values (max 252). A parked car contributes ONE keyframe however long it stands,
so the board seen only from the parking spot is one scan's worth; driving past it is
what fills it in. Every run writes `median_spacing_m` and the bright-point fraction to
`glim_report.yaml`.
