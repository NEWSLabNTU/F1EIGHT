# Copyright 2022 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Topic rate monitors for the coach-pursuit lab (Lab 2), without the
# component_state_monitor's StateMonitor.
#
# autoware_component_state_monitor's component_state_monitor.launch.py does two
# things: it loads one topic_state_monitor per row of a topic list, and it
# starts StateMonitor, which reads every /diagnostics message and republishes
# per-module ModeChangeAvailable on
# /system/component_state_monitor/component/{launch,autonomous}/<module>. In
# Autoware 1.5.0 the only reader of those is the ADAPI AutowareStateNode
# (/autoware/state), which the lab API component does not run. The rate
# monitors are what the lab needs: the diagnostic graph reads their
# diagnostics directly, by node name.
#
# This is that launch file minus StateMonitor. Same topic list format, same
# container name and namespace, same monitor node names and diag names, so
# autosdv-pursuit-main.yaml resolves unchanged. Re-fork from
# component_state_monitor.launch.py when the Autoware version changes.
from pathlib import Path

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace
import yaml


def create_topic_monitor_node(row, target_container):
    """Include one topic_state_monitor for a row of the topic list."""
    tf_mode = '' if 'topic_type' in row['args'] else '_tf'
    package = FindPackageShare('autoware_topic_state_monitor')
    include = PathJoinSubstitution(
        [package, f'launch/load_topic_state_monitor{tf_mode}.launch.xml']
    )
    diag_name = '{}_topic_status'.format(row['module'])
    arguments = [('diag_name', diag_name), ('target_container', target_container)] + [
        (k, str(v)) for k, v in row['args'].items()
    ]
    return IncludeLaunchDescription(include, launch_arguments=arguments)


def launch_setup(context, *args, **kwargs):
    """Create the monitor container and one monitor per row for this run mode."""
    target_namespace = context.launch_configurations.get('ros_namespace', None)
    target_container = make_namespace_absolute(
        prefix_namespace(target_namespace, 'component_state_monitor/container')
    )
    mode = LaunchConfiguration('mode').perform(context)
    rows = yaml.safe_load(Path(LaunchConfiguration('file').perform(context)).read_text())
    rows = [row for row in rows if mode in row['mode']]
    container = ComposableNodeContainer(
        namespace='component_state_monitor',
        name='container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
    )
    return [container, *[create_topic_monitor_node(row, target_container) for row in rows]]


def generate_launch_description():
    """Topic list file and run mode, then launch_setup."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument('file'),
            DeclareLaunchArgument('mode'),
            OpaqueFunction(function=launch_setup),
        ]
    )
