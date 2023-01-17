# Copyright 2023 ICube Laboratory, University of Strasbourg
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

#
# Author: Adnan SAOOD

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

description_package = "polaris_description"
description_file = "polaris.config.xacro"
polaris_runtime_config_package = "polaris_bringup"
controllers_file = "polaris_broadcaster.yaml"


def launch_setup(context, *args, **kwargs):
    urdf_path = os.path.join(
        get_package_share_directory('polaris_description'),
        'config',
        'polaris.config.xacro')

    desc_file = xacro.parse(open(urdf_path))
    xacro.process_doc(desc_file)
    polaris_description = {"robot_description": desc_file.toxml()}

    polaris_controller_config = os.path.join(
        get_package_share_directory("polaris_bringup"),
        "config",
        "polaris_broadcaster.yaml")

    polaris_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        emulate_tty=True,
        parameters=[polaris_description, polaris_controller_config],
        output={"screen"},
    )

    nodes_to_start = [
        polaris_control_node,
    ]

    return nodes_to_start


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
