# autosdv_marker_survey

Map the ArUco markers around the car with its ZED X Mini, and write the Lab 2
course they define. Place the boards wherever the hall allows, name one of
them as the origin, drive or push the car past all of them, stop: the course
(`course.yaml`, the lanelet2 marker map, the localizer's marker settings) is
written for you.

```bash
just lab2 survey courses/my_hall 24          # origin: marker 24 (an LCTK board)
#   ...drive slowly past every marker; the log names each new one...
#   Ctrl-C, or in another terminal: just lab2 survey-save
just lab2 course-map courses/my_hall         # coverage check, as for any course
just lab2 camera-check courses/my_hall       # car in the start box: verify
```

With a display (`DISPLAY` set), `just lab2 survey` also opens **rqt_image_view
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
`dictionary:=ARUCO_MIP_36h12 marker_size:=0.25` for the course's printed
markers (default `DICT_5X5_1000`, 0.48 m: the LCTK calibration boards).

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
4. The result is expressed in the **course frame of the origin marker**:
   - z up: the floor's normal, from the car's `base_link` poses;
   - the origin is on the floor below the origin marker's centre;
   - x points the way the origin marker faces.

   `course.yaml` gets every marker's `id, x, y, z, yaw, pitch, roll`, plus
   `marker_size` and `dictionary`. A board is rarely exactly upright, and the
   localizer reads each marker's full orientation from its corners, so the
   measured tilt goes into the map: `pitch` is the top leaning back, `roll`
   the board turned in its own plane, counter-clockwise from the front (both
   radians; `autosdv_waypoint_lab/course.py` defines them). Left out, a 3°
   tilt alone would put every fix from that marker about 5 cm off at 1 m. A new course also gets the recording's start as its
   `start_pose`; an existing `course.yaml` keeps everything but those fields.
   Write `path.csv` in this same frame.

`survey_report.yaml` lists each marker's views, reprojection error, 1σ position
and yaw uncertainty, and tilt from vertical (also split into `pitch_deg` and
`roll_deg`). It flags markers seen fewer than 3
times, with more than 5 cm or 1° of uncertainty, or tilted more than 10°.
`survey_data.npz` keeps the recording's keyframes and detections, so
`just lab2 survey-solve courses/my_hall [--origin ID]` can solve it again
without the car.

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
| Rendered images played through the node (`lab2_marker_survey_synthetic_bag`, one lap at 3 Hz, half resolution) | median 2.9 cm, max 4.5 cm | max 0.5° |

Not yet measured on the real ZED; compare a real survey against tape
measurements with `lab2_marker_survey_compare COURSE_DIR truth.yaml`.

## Without the car

```bash
ros2 run autosdv_marker_survey lab2_marker_survey_synthetic_bag tmp/synth \
    --course src/lab/autosdv_waypoint_lab/config/courses/hall_demo --origin 10
ros2 launch autosdv_marker_survey marker_survey.launch.xml origin_id:=10 \
    course_dir:=$PWD/tmp/synth/course launch_camera:=false use_sim_time:=true &
ros2 bag play tmp/synth/bag --clock
just lab2 survey-save
ros2 run autosdv_marker_survey lab2_marker_survey_compare tmp/synth/course tmp/synth/truth.yaml
```

## Notes

- The image must be rectified (the ZED's `rgb/color/rect/image` is);
  `camera_info`'s projection matrix is used and distortion is ignored.
- `pose_source:=tf` uses TF `world_frame -> image frame` instead of the ZED
  odometry, for example the car's EKF.
- DICT_ARUCO_MIP_36h12 needs OpenCV ≥ 4.7; DICT_5X5_1000 works with any.
