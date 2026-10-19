# autosdv_marker_survey

Map the ArUco markers around the car with its ZED X Mini, and write the marker
map they define. Place the boards wherever the hall allows, name one of them as
the origin, drive or push the car past all of them, stop: the map
(`course.yaml`, the lanelet2 marker map, the localizer's marker settings) is
written for you, in the **origin board's frame**, and
`autosdv_aruco_localization` localizes on it. Lab-independent; the user-facing
tool is `just aruco-map` (`docs/guides/aruco-map.md`).

That is the survey (Lab 2 calls it stage 1): it fixes where the boards stand
relative to one another. **Optionally**, `just aruco-map align --route NAME`
puts the result into a taped route's start-box frame from one straight drive
along the route's first leg (`autosdv_marker_survey/align.py`). Lab 2 does that
for its graded route (`just lab2 survey-align`; the field procedure is in
`docs/guides/lab2-waypoint-stack.md`, "On the field").

```bash
just aruco-map survey 24 courses/my_hall     # origin: marker 24 (one of the lab boards)
#   ...drive slowly past every marker; the log names each new one...
#   Ctrl-C, or in another terminal: just aruco-map save
just aruco-map check courses/my_hall         # drive by hand, watch the localization
```

Ctrl-C ends the recording: the node writes `survey_data.npz`, then solves. A long
recording's bundle adjustment can outlast play_launch's shutdown (2 s + 5 s,
then SIGKILL); `just aruco-map` then solves again from `survey_data.npz`
(`marker_survey_solve`), so the map is written either way.

With a display (`DISPLAY` set), `just aruco-map survey` also opens **rqt_image_view
on `/marker_survey/debug/image`**: the processed frames (5 Hz) with every
detection drawn.
- **Green:** a known marker, labelled `id (views)`.
- **Yellow:** a marker seen for the first time.
- **Red:** too small to use.

The origin is outlined thicker, and a dot marks each marker's top-left corner.
A status bar shows the marker and keyframe counts, `[KEYFRAME]` when a frame
is kept, and whether the origin has been seen. Without a display, or with
`viewer:=false`, nothing opens. Over SSH, run `rqt_image_view
/marker_survey/debug/image` on a machine on the same network and ROS domain;
the image is published only while someone watches.

Arguments after the origin go to the launch file:
`dictionary:=ARUCO_MIP_36h12 marker_size:=0.25` for the printed markers
(default `DICT_5X5_1000`, 0.38 m: the lab's 500 mm boards), and
`vehicle_id:=car_c` for that car's camera mount (default `sim`, the nominal
car).

**`marker_size` is the edge of the black square, not of the board, and it must
be measured** with a ruler on every kind of board in use. A wrong size scales
every distance in the map. Two boards seen so far:

| board | plate | black square (`marker_size`) |
| --- | --- | --- |
| LCTK 600 mm calibration board | 0.60 m | 0.48 m |
| **the lab's boards** (Lab 2) | 0.50 m | **0.38 m (the default)**; marker centre 0.25 m above the floor |

**The floor.** `z` is the marker centre above the floor, and the floor is
taken to be at `base_link` (the mean of the car's poses). A car held up on a
stand, wheels off the floor, puts the floor too high: every `z` comes out low
by the stand's height. Pass `floor_offset:=0.12` (metres the floor lies below
`base_link`) to correct it, or `marker_survey_solve ... --floor-offset
0.12` afterwards. x, y and yaw do not depend on it. The survey warns, and flags
the marker in `survey_report.yaml`, when a marker centre is below half the
marker size above the floor: the causes are the car on a stand, a wrong camera
mount TF, or a board that is not on the floor. A stand that is itself tilted
also tilts the floor normal, and with it every pitch and tilt: level the car
or survey it on the floor.

**Stationary car.** `just aruco-map save` with one keyframe (a car that has not moved)
skips the bundle adjustment: each marker is that one view's estimate, with no
uncertainty (`sd` null) and the report says `single keyframe, no bundle
adjustment`. A quick look at the boards, not a survey.

**OpenCV.** JetPack 6 ships OpenCV 4.5.4; the survey uses its function API
there (`DetectorParameters_create`, `detectMarkers`) and the `ArucoDetector`
class on 4.7 and later.

## How it works

1. The ZED runs with **positional tracking** on (`config/zed_survey.yaml`): its
   visual-inertial odometry gives every frame a camera pose. It drifts slowly,
   about 1 % of the distance driven, but it links frames in which markers are
   seen apart.
2. `marker_survey` processes frames at 5 Hz. It detects markers (OpenCV ArUco,
   sub-pixel corners), and keeps a frame as a keyframe when the car has moved
   15 cm or turned 5°, or a new marker appears. Each marker is placed in the map
   as soon as it is first seen: the log says `new marker 7, 3.2 m away`, and
   `~/markers` shows the live map in RViz.
3. On save it **bundle-adjusts** everything. The unknowns are every keyframe pose
   and every marker pose. The residuals are the marker corners' reprojection
   (robust, 0.5 px) and the tracking's motion between consecutive keyframes,
   weighted for its drift. Markers seen together correct the drift; markers
   seen alone still land in the frame.
4. The result is expressed in the **origin board's frame**:
   - z up: the floor's normal, from the car's `base_link` poses;
   - the origin is on the floor below the origin marker's centre;
   - x points the way the origin marker faces (out of its printed face,
     towards a viewer), made horizontal;
   - y = z × x: to the right of someone facing the origin board (along the
     board's own x axis), so the origin board itself has yaw 0.

   `course.yaml` gets every marker's `id, x, y, z, yaw, pitch, roll`, plus
   `marker_size` and `dictionary`. A board is rarely exactly upright, and the
   localizer reads each marker's full orientation from its corners, so the
   measured tilt goes into the map: `pitch` is the top leaning back, `roll`
   the board turned in its own plane, counter-clockwise from the front (both
   radians; `autosdv_aruco_localization/marker_map.py` defines them). A board mounted
   rotated by 90° in its plane has `roll` ±90° and is not tilted: the
   report's `tilt_deg` is the lean, the angle between the marker's normal and
   the horizontal plane, and ignores `roll`. Left out, a 3°
   tilt alone would put every fix from that marker about 5 cm off at 1 m. A new map also gets the recording's start as its
   `start_pose` (where `start_pose_init` seeds the EKF); an existing
   `course.yaml` keeps everything but those fields.

`survey_report.yaml` lists each marker's views, reprojection error, 1σ position
and yaw uncertainty, and lean from vertical (`tilt_deg`; the orientation also split into `pitch_deg` and
`roll_deg`). It flags markers seen fewer than 3
times, with more than 5 cm or 1° of uncertainty, or tilted more than 10°.
`survey_data.npz` keeps the recording's keyframes and detections, so
`just aruco-map solve courses/my_hall [--origin ID]` can solve it again
without the car (another `--origin` re-expresses the map in that board's frame).

## Driving the survey

- Slowly, about 0.3 m/s; push the car or use the transmitter. The stack is not
  running, so nothing brakes for you.
- Pass every marker from two or three positions, and turn the car toward groups
  of markers so that several share a frame: shared frames are what correct the
  drift.
- End back where you started if you can: the closed loop helps.

## Accuracy

| Test | Marker position | Yaw |
| --- | --- | --- |
| Simulated corners, two laps of a 10 × 4 m oval (11 boards, 0.48 m, ZED intrinsics), tracking drift 1 % | median 1.0 cm, max 4.0 cm | max 1.3° |
| The same, drift 2 % | median 1.2 cm, max 3.8 cm | max 1.0° |
| Rendered images played through the node (`marker_survey_synthetic_bag`, one lap at 3 Hz, half resolution) | median 2.9 cm, max 4.5 cm | max 0.5° |
| The same bag through `just aruco-map survey 10 ... launch_camera:=false use_sim_time:=true`, Ctrl-C, the map re-solved from `survey_data.npz` (hall_demo, 11 boards, 179 keyframes) | median 3.4 cm, max 4.9 cm | max 1.3° |

Not yet measured on the real ZED; compare a real survey against tape
measurements with `marker_survey_compare COURSE_DIR truth.yaml`.

## Without the car

```bash
ros2 run autosdv_marker_survey marker_survey_synthetic_bag tmp/synth \
    --course src/lab/autosdv_waypoint_lab/config/courses/hall_demo --origin 10
just aruco-map survey 10 tmp/synth/course launch_camera:=false use_sim_time:=true &
ros2 bag play tmp/synth/bag --clock
kill -INT %1          # Ctrl-C: the survey solves (or `just aruco-map save`)
ros2 run autosdv_marker_survey marker_survey_compare tmp/synth/course tmp/synth/truth.yaml
```

## Notes

- The image must be rectified (the ZED's `rgb/color/rect/image` is);
  `camera_info`'s projection matrix is used and distortion is ignored.
- `pose_source:=tf` uses TF `world_frame -> image frame` instead of the ZED
  odometry, for example the car's EKF.
- DICT_ARUCO_MIP_36h12 needs OpenCV ≥ 4.7, so on JetPack 6 (OpenCV 4.5.4) only the
  5X5 boards work; DICT_5X5_1000 works with any.
- The ZED publishes its odometry BEST_EFFORT; the node subscribes the same way.
