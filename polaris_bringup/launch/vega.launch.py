# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

description_package = "polaris_description"
description_file = "polaris.config.xacro"
polaris_runtime_config_package = "polaris_bringup"
controllers_file = "polaris_broadcaster.yaml"

def launch_setup(context, *args, **kwargs):


    polaris_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "config", description_file]),
            
        ]
    )
    polaris_description = {"robot_description": polaris_description_content}




    polaris_braodcaster = PathJoinSubstitution(
        [FindPackageShare(polaris_runtime_config_package), "config", controllers_file]
    )

    polaris_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[polaris_description, polaris_braodcaster],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )



    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # io_and_status_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["io_and_status_controller", "-c", "/controller_manager"],
    # )


    nodes_to_start = [
        polaris_control_node,
        # joint_state_broadcaster_spawner,
        # io_and_status_controller_spawner,
        
    ]

    return nodes_to_start


def generate_launch_description():
   

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
