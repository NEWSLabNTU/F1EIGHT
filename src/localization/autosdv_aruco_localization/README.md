# autosdv_aruco_localization

ArUco localization on a surveyed marker map: the car's ZED sees the boards,
Autoware's AR-tag localizer turns each sighting into a pose fix, and Autoware's
EKF fuses the fixes with wheel and gyro odometry. Lab-independent: a map comes
from `autosdv_marker_survey` (`just aruco-map survey`), and Lab 2
(`autosdv_waypoint_lab`) runs its waypoint stack on this package, never the
reverse. The user guide is `docs/guides/aruco-map.md`.

## What is here

| file | what it is |
| --- | --- |
| `autosdv_aruco_localization/marker_map.py` | the marker map: `course.yaml` (boards, `start_pose`, `marker_size`, `dictionary`) to `lanelet2_map.osm` (one `pose_marker` polygon per board) and `ar_tag.param.yaml`; `make_marker_map DIR` regenerates them. No ROS |
| `launch/aruco_localization.launch.xml` | map loader, AR-tag localizer (AutoSDV's fork, with `dictionary`), vehicle velocity converter, gyro odometer, EKF, twist2accel, `start_pose_init` |
| `launch/zed_aruco.launch.xml`, `config/zed_aruco.yaml` | the ZED X Mini driver for ArUco: the rectified left image and the IMU, depth and tracking off |
| `config/ar_tag_based_localizer.param.yaml` | the localizer's covariance, range limit and EKF gate |
| `config/dictionaries/` | OpenCV families the aruco library lacks, as files (`DICT_5X5_1000`, the lab's boards) |
| `start_pose_init` | seeds the EKF at the map's `start_pose` and triggers it: the localizer publishes nothing until the EKF has a pose to gate against |
| `loc_check` (+ `launch/loc_check.launch.xml`, `rviz/loc_check.rviz`) | the localization check: the car's trail, the marker fixes, the map's route if it has one, and a summary of fix rate, blind gaps and fix-vs-odometry disagreement. `/loc_check/path`, `/loc_check/trail`, `/loc_check/fixes` |
| `aruco_camera_check MAP_DIR` | car still at the start pose: fix quality, and each board's detections against its surveyed place |
| `mock_aruco_camera` | a stand-in ZED for testing without the car: renders the map's boards from a simulated car's true pose, so the real localizer runs |
| `virtual_marker_localizer`, `marker_sim.py` | a stand-in for camera + localizer together: fixes from the true pose with a noise model; `marker_sim.coverage` scores how well boards cover a path |

## Decisions: what moved here from Lab 2, and what stayed

Moved (ArUco localization, nothing lab-specific): the map files and their
generator (split out of `autosdv_waypoint_lab/course.py`, which re-exports
them), the localization launch (was `waypoint_localization.launch.xml`), the
ZED launch and config, the localizer parameters, the dictionaries,
`start_pose_init`, `loc_check`, the camera check, `mock_aruco_camera`,
`virtual_marker_localizer` and `marker_sim`.

Stayed in `autosdv_waypoint_lab`: `localization_monitor` (it judges the path
deviation from Lab 2's `WaypointProgress`), the routes and their coverage
report (`make_course_map`), `bench_straight_driver` and the vehicle bench, the
grading. `autosdv_waypoint_lab/config/dictionaries/DICT_5X5_1000.dict` is kept
as a link to this package's file, because `ar_tag.param.yaml` files generated
before the move name that path.

## Frame

A map is in whatever frame its `course.yaml` is in. A survey writes the origin
board's frame (origin on the floor below that board's centre, x the way it
faces, z up, y = z × x); `just aruco-map align` rewrites a survey into a route's
start-box frame. Nothing in this package depends on which.
