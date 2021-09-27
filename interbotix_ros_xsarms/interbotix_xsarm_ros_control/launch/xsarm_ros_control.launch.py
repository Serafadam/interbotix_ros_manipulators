# Copyright 2020 RT Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('interbotix_xsarm_descriptions'),
        'urdf',
        'rx200.urdf.xacro')
    model = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " "
            "robot_name:=",
            "rx200",
            " ",
            "base_link_frame:=",
            "base_link",
            " ",
            "show_ar_tag:=",
            "false",
            " ",
            "show_gripper_bar:=",
            "false",
            " ",
            "show_gripper_fingers:=",
            "false",
            " ",
            "use_world_frame:=",
            "true",
            " ",
            "external_urdf_loc:=",
            "''",
            " ",
            "load_gazebo_configs:=",
            "false",
        ]
    )
    robot_description = {"robot_description": model}

    crane_plus_controllers = os.path.join(
        get_package_share_directory('interbotix_xsarm_ros_control'),
        'config',
        'ros2_config.yaml'
        )
    print(crane_plus_controllers)

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace="rx200",
        parameters=[robot_description, crane_plus_controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    spawn_arm_controller = Node(
                package="controller_manager",
                namespace="rx200",
                executable="spawner",
                arguments=["arm_controller", "-c", "/rx200/controller_manager"],
                )
    

    spawn_gripper_controller = Node(
                package="controller_manager",
                executable="spawner",
                namespace="rx200",
                arguments=["gripper_controller", "-c", "/rx200/controller_manager"],
                )

    return LaunchDescription([
      controller_manager,
      spawn_arm_controller,
      spawn_gripper_controller
    ])