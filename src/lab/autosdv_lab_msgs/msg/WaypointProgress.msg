# The waypoint course as waypoint_path_publisher sees it, once per cycle.
# For bags and grading: laps and finish come from here.

std_msgs/Header header      # stamp = publish time; frame_id = map
float32 progress            # m driven along the course (unwrapped across laps)
float32 lap_length          # m
float32 run_length          # m, lap_length * laps (closed) or the path length (open)
uint16  laps_done
uint16  laps                # laps in the run
bool    finished            # progress reached the end of the run
float32 distance_to_path    # m, the car's distance from the course polyline
