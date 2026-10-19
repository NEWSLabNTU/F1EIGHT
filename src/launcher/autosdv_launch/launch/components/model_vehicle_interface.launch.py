"""Include a vehicle model's vehicle_interface.launch.xml with ITS OWN speed controller.

    <include file="$(find-pkg-share autosdv_launch)/launch/components/model_vehicle_interface.launch.py">
      <arg name="vehicle_interface_launch" value="$(find-pkg-share X_launch)/launch/vehicle_interface.launch.xml"/>
      <arg name="vehicle_id" value="..."/>   ...any of FORWARDED
    </include>

Why a Python file: ROS 2 launch configurations leak into an included file. A caller that
declares `speed_controller_class` (waypoint_follow.launch.xml does, default "": "the model's
own") hands that "" to the model's file, where it beats the model's own
`<arg name="speed_controller_class" default="teamN_vehicle_interface.speed_pid:SpeedPID">`
(a default applies only to a configuration that is not set). The actuator then gets "" and
falls back to its built-in AckermannSpeedPID: the team's SpeedPID never runs. XML has no
way to unset a configuration in Humble; UnsetLaunchConfiguration does it here, scoped, so the
model's defaults apply.

The arguments in FORWARDED are passed on by name, explicitly (play_launch's parser does not
let configurations leak, so the model's required vehicle_id must be given); one the caller
leaves out is not passed, and the model's own default applies.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, UnsetLaunchConfiguration)
from launch.launch_description_sources import AnyLaunchDescriptionSource

#: the caller's names for "use the model's own": unset so the model's defaults win
MODEL_OWNED = ('speed_controller_class', 'speed_controller_params')
#: what waypoint_follow.launch.xml hands a model's vehicle interface
FORWARDED = ('hardware', 'launch_vehicle_interface', 'is_simulation', 'vehicle_id',
             'speed_controller_dt_source', 'emergency_stop_on_any_mrm',
             'publish_operation_mode_availability', 'launch_signal_manager',
             'launch_velocity_report', 'actuator_param_overrides', 'actuator_log_level',
             'vehicle_info_param_file')


def _include(context):
    cfg = context.launch_configurations
    args = [(k, cfg[k]) for k in FORWARDED if k in cfg]
    return [GroupAction(scoped=True, actions=[
        *[UnsetLaunchConfiguration(name) for name in MODEL_OWNED],
        IncludeLaunchDescription(AnyLaunchDescriptionSource(cfg['vehicle_interface_launch']),
                                 launch_arguments=args),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_interface_launch',
                              description="the model's launch/vehicle_interface.launch.xml"),
        OpaqueFunction(function=_include),
    ])
