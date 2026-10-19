# ArUco map: survey the boards, localize on them (`just aruco-map`)

`just aruco-map` builds a map of the ArUco boards in a hall from one drive of
the car's ZED X Mini, and checks the ArUco localization on that map. It belongs
to no lab: its settings come from the command line only, and nothing in it
drives the car. Lab 2 uses it through its own recipes (`just lab2 survey`,
`survey-align`, `loc-check`), which add the lab's car and route.

| command | what it does |
| --- | --- |
| `just aruco-map survey ORIGIN_ID [DIR] marker_size:=M [key:=value ...]` | scan: drive past every board, Ctrl-C solves the map, in the origin board's frame |
| `just aruco-map survey ORIGIN_ID [DIR] marker_size:=M --route NAME` | the same, then align it to a route's start box (below) |
| `just aruco-map scan ORIGIN_ID [DIR] marker_size:=M [key:=value ...]` | the scan alone (`DIR ORIGIN_ID` also works, Lab 2's old order) |
| `just aruco-map align [DIR] --route NAME [--vehicle-id car_X] [--force]` | optional: put a survey into a route's start-box frame |
| `just aruco-map solve [DIR] [--origin ID] [--floor-offset M]` | solve the scan again from `DIR/survey_data.npz`, no car |
| `just aruco-map save` | solve and write the running scan now, without stopping it |
| `just aruco-map check [DIR] [vehicle_id:=car_X] [rviz:=false]` | drive by hand and watch where the localizer puts the car |

`DIR`: `survey` and `scan` write a new `surveys/<date>_<time>/` unless one is
named, and point `surveys/latest` at it; `align`, `solve` and `check` read
`surveys/latest` unless one is named. Relative paths are from the repository
root.

## Settings

All on the command line, as `key:=value`:

| key | default | what |
| --- | --- | --- |
| `vehicle_id` | `sim` | the car's instance, `individual_params/config/<id>`: the camera mount (URDF) and the IMU calibration. `sim` is the nominal car. Nothing drives, so the default is never unsafe, only less exact than the car's own |
| `dictionary` | `DICT_5X5_1000` | the printed boards' family (`ARUCO_MIP_36h12` needs OpenCV 4.7 or newer: not on JetPack 6) |
| `marker_size` | **required** for a scan | metres, the edge of the **black square**, not of the board. Measure it: a wrong size scales every distance by the same factor, so there is no default. A 7×7-cell marker with one cell of white margin on a W board: W × 7/9 (0.60 m board: 0.467). A scan taken at the wrong size: `just aruco-map solve DIR marker_size:=X` re-solves it |
| `floor_offset` | `0` | metres the floor lies below `base_link`, for a car on a stand |

Any other `key:=value` goes to the launch file the command runs
(`marker_survey.launch.xml`, `aruco_loc_check.launch.xml`). `align` takes
options instead: `--route` (required), `--vehicle-id` (or `vehicle_id:=`),
`--force`, `--bench --truth DIR`.

## The frame

A survey's map is in the **origin board's frame**, the board named by
`ORIGIN_ID`:

- **origin**: on the floor, straight below the centre of the origin board's
  black square;
- **z**: up, the floor's normal, fitted to the car's own `base_link` poses
  during the drive (so a board that leans does not tilt the map);
- **x**: the direction the origin board faces (out of its printed face,
  towards someone looking at it), made horizontal;
- **y** = z × x: to the right of someone standing in front of the origin board
  and facing it (the board's own x axis).

So the origin board itself sits at (0, 0, its height) with yaw 0. Every board
is stored with `x, y, z, yaw, pitch, roll` (radians): `yaw` the way it faces,
`pitch` its top leaning back, `roll` turned in its own plane,
counter-clockwise from the front. Pick as origin a board that is well in view
and stands where you want the map's origin; any board works, and
`just aruco-map solve DIR --origin ID` re-expresses a recording in another
board's frame.

The map's `start_pose` is where the recording started (`base_link`, in this
frame); `just aruco-map check` and Autoware's EKF start there, so begin the
check with the car where the scan began.

## Outputs (`DIR/`)

| file | |
| --- | --- |
| `course.yaml` | the boards, `marker_size`, `dictionary`, `start_pose`, and a `survey` block (origin board, date, frame, keyframes, reprojection RMS). Named after Lab 2's course file, which is this plus a route |
| `lanelet2_map.osm` | one lanelet2 `pose_marker` polygon per board, what the AR-tag localizer reads |
| `ar_tag.param.yaml` | the localizer's `dictionary`, `marker_size`, `target_tag_ids` |
| `map_projector_info.yaml` | `projector_type: local` |
| `survey_report.yaml` | per board: views, reprojection error, 1σ position and yaw, tilt, flags (seen under 3 times, over 5 cm or 1° uncertain, tilted over 10°) |
| `survey_data.npz` | the recording (keyframes, detections): `solve` works from it |

After `align`, the same directory holds the map in the route's frame instead,
with the route's `path.csv` and an `alignment` block, plus `align_report.txt`,
`align_poses.csv`, `align_pnp.csv` and the survey's own result in `stage1/`.

## Surveying

```bash
just aruco-map survey 24 marker_size:=0.467    # origin: board 24; surveys/<date>_<time>
just aruco-map survey 24 courses/my_hall vehicle_id:=car_c marker_size:=0.25
```

Drive or push the car slowly (about 0.3 m/s) past every board, from two or
three positions each, turning toward groups of boards so that several share a
frame; end where you started if you can. The log names each new board. Then
Ctrl-C. The last line before just's own `error: interrupted by SIGINT` is the
verdict: the number of boards, or a refusal when fewer than two were found or
the origin was never seen.

Ctrl-C makes the survey node write `survey_data.npz` and then solve (a bundle
adjustment of every keyframe and board). A long recording takes longer than
play_launch waits before it kills a node, so when the map was not written the
script solves again from `survey_data.npz` itself; the result is the same.

With a display, `rqt_image_view` shows the detections. The ZED itself never
gets a `DISPLAY` nor VirtualGL's `LD_PRELOAD` faker (Argus fails with either on
a VNC desktop); `scripts/camera_display.sh` takes them out of the launch and
hands a working display to the viewers only.

## Checking the localization

```bash
just aruco-map check                           # surveys/latest, the nominal car
just aruco-map check courses/my_hall vehicle_id:=car_c
```

The ZED, the AR-tag localizer and the EKF on the map
(`autosdv_aruco_localization`), the actuator as a dry run: nothing can drive
the car. Start with the car at the map's `start_pose` (where the scan began),
then drive it by hand or by transmitter. RViz shows the trail (red), the marker
fixes (green) and, for a map with a route, the route (white); `loc_check` logs
fix rate, the longest stretch without a fix, how far each fix lands from the
EKF's odometry, and the distance from the route, every 10 s and into
`DIR/loc_check_summary.txt`.

## Aligning to a route (optional)

A survey is a complete map for localization. A **route** has its own frame:
a directory with a `course.yaml` (`path`, `start_pose` (0, 0, 0)) and its
`path.csv`, taped on the floor with the start box at the origin and the first
leg along +x. `align` puts the survey into that frame:

```bash
just aruco-map align --route lab2_uturn                 # surveys/latest
just aruco-map align surveys/<dir> --route path/to/route --vehicle-id car_c
just aruco-map survey 24 marker_size:=0.467 --route lab2_uturn   # scan, then align, guided
```

`--route` is a directory, or a NAME found as `config/courses/NAME` in an
installed package (Lab 2 ships `lab2_uturn`, `lab2_straight_2m`, ... in
`autosdv_waypoint_lab`). Put the car in the taped start box facing along the
first leg; the tool seeds the EKF from the boards in view and prints READY;
drive straight along the first leg, 2 m or more, stop, Ctrl-C. The start pose
becomes the origin, the drive's fitted line +x; the boards keep their heights
and tilts. `align_report.txt` says how straight the drive was and what that
means for the heading. `marker_survey_align_solve DIR --route NAME [--force]`
re-solves a recorded drive without the car.

Lab 2: `just lab2 survey-align` is this with the lab's route and car, and
`just lab2 run` reads `surveys/latest` once it is aligned
(`docs/guides/lab2-waypoint-stack.md`, "On the field").

## Without the car

```bash
# a rendered recording of a known hall, its truth, and the survey of it
ros2 run autosdv_marker_survey marker_survey_synthetic_bag tmp/synth \
    --course src/lab/autosdv_waypoint_lab/config/courses/hall_demo --origin 10
just aruco-map survey 10 tmp/synth/map marker_size:=0.38 launch_camera:=false use_sim_time:=true &
ros2 bag play tmp/synth/bag --clock
kill -INT %1
ros2 run autosdv_marker_survey marker_survey_compare tmp/synth/map tmp/synth/truth.yaml
```

Measured 2026-10-20 (hall_demo, 11 boards, 179 keyframes): median 3.4 cm,
max 4.9 cm, yaw max 1.3°.

The localization runs on Lab 2's vehicle bench with `mock_aruco_camera`, which
renders a map's boards from the simulated car's true pose for the real
localizer: `just lab2 bench camera:=mock course_dir:=DIR`, and
`loc_check.launch.xml course_dir:=DIR` beside it. On `lab2_uturn` at rest:
15 fixes/s, fix vs odometry p95 0.5 cm. The alignment on the bench:
`python3 scripts/testing/lab2/align_bench.py --run` (Lab 2's harness).

## Where things are

- `just/aruco_map.just`, `scripts/aruco_map/aruco_map.sh` (the commands),
  `scripts/aruco_map/survey_align.py` (the alignment's driver),
  `scripts/camera_display.sh`;
- `src/localization/autosdv_marker_survey` (the survey node, the solver, the
  alignment maths: its README has the method and the accuracy);
- `src/localization/autosdv_aruco_localization` (the map files, the
  localization launch, `start_pose_init`, `loc_check`, the camera stand-ins);
- `src/launcher/autosdv_launch/launch/aruco_loc_check.launch.xml`,
  `aruco_survey_align.launch.xml` (the car-side stacks: the sensor kit with its
  drivers off, the dry-run vehicle interface).

Not yet done on a car: the commands were exercised on recordings, the bench
and stand-ins; the survey and alignment were last run on a car as Lab 2's
`just lab2 survey` / `survey-align`, which now run these.
