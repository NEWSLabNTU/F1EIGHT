# One cycle of path_tracker, for bags and grading.
#
# steering_raw / speed_raw are what the student PathTracker returned, before
# the shell's clamps (NaN when it raised or was disabled); steering / speed
# are what was sent. limit names the clamp that bound (steer, path_speed),
# fault says why the law's output was not used.

std_msgs/Header header      # stamp = control time; frame_id = map
bool    autonomous          # operation mode was AUTONOMOUS this cycle
float32 x                   # m, localized base_link
float32 y
float32 yaw                 # rad
float32 speed               # m/s, from /localization/kinematic_state
float32 cross_track         # m, signed, left of the path positive
float32 heading_error       # rad, car yaw minus path yaw at the projection
float32 path_speed          # m/s, the waypoint speed at the projection
float32 steering_raw        # rad
float32 speed_raw           # m/s
float32 steering            # rad
float32 speed_command       # m/s
string  limit
string  fault
bool    disabled
