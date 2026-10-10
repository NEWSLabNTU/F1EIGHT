"""Lab 2 field calibration: the measurements the first field session takes.

ROS-free logic, one module per measurement, each with a thin CLI:

  speed_steps   the target-speed schedule the speed_step_planner node drives
  motor_fit     MotorPlantParams from a bag of PWM and wheel speed (lab2_fit_motor)
  braking       reaction time and deceleration per stop (lab2_braking_report)
  bagio         reading a rosbag2 directory (the only module that needs ROS)

The detector report lives with the detector, in reflector_pose_init
(reflective_pose_core.tracking_report, CLI board_tracking_report).
"""
