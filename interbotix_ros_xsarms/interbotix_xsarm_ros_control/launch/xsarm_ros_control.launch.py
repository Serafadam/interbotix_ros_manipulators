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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def launch_setup(context, *args, **kwargs):

    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")
    base_link_frame = LaunchConfiguration("base_link_frame")
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    show_gripper_bar = LaunchConfiguration("show_gripper_bar")
    show_gripper_fingers = LaunchConfiguration("show_gripper_fingers")
    use_world_frame = LaunchConfiguration("use_world_frame")
    external_urdf_loc = LaunchConfiguration("external_urdf_loc")
    load_gazebo_configs = LaunchConfiguration("load_gazebo_configs")
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    dof = LaunchConfiguration("dof")
    model = LaunchConfiguration("model")

    urdf_path = PathJoinSubstitution([get_package_share_directory("interbotix_xsarm_descriptions"), "urdf", robot_model])

    model = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            ".urdf.xacro" " ",
            "robot_name:=",
            robot_name,
            " ",
            "base_link_frame:=",
            base_link_frame,
            " ",
            "show_ar_tag:=",
            show_ar_tag,
            " ",
            "show_gripper_bar:=",
            show_gripper_bar,
            " ",
            "show_gripper_fingers:=",
            show_gripper_fingers,
            " ",
            "use_world_frame:=",
            use_world_frame,
            " ",
            "external_urdf_loc:=",
            external_urdf_loc,
            " ",
            "load_gazebo_configs:=",
            load_gazebo_configs,
        ]
    )



    robot_description = {"robot_description": model}

    controllers = os.path.join(
        get_package_share_directory('interbotix_xsarm_ros_control'),
        'config',
        f'{dof.perform(context)}dof_controllers.yaml'
        )
    control_prefix = os.path.join(get_package_share_directory("interbotix_xsarm_control"))
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name,
        parameters=[robot_description, controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [control_prefix, "launch", "xsarm_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_name": robot_name,
            "base_link_frame": base_link_frame,
            "show_ar_tag": show_ar_tag,
            "show_gripper_bar": show_gripper_bar,
            "show_gripper_fingers": show_gripper_fingers,
            "use_sim": use_sim,
            "use_world_frame": use_world_frame,
            "external_urdf_loc": external_urdf_loc,
            "use_rviz": use_rviz,
        }.items(),
    )


    spawn_arm_controller = Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["arm_controller", "-c", f"/{robot_name.perform(context)}/controller_manager"],
                )
    

    spawn_gripper_controller = Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["gripper_controller", "-c", f"/{robot_name.perform(context)}/controller_manager"],
                )

    return [
      controller_manager,
      spawn_arm_controller,
    #   spawn_gripper_controller,
    #   control,  
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_model",
            default_value='""',
            description="model type of the Interbotix Arm such as 'wx200' or 'rx150'",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value='""',
            description="name of the robot (typically equal to robot_model, but could be anything)'",
        ),
        DeclareLaunchArgument(
            "base_link_frame",
            default_value="'base_link'",
            description="name of the 'root' link on the arm; typically 'base_link', but can be changed if attaching the arm to a mobile base that already has a 'base_link' frame",
        ),
        DeclareLaunchArgument(
            "show_ar_tag",
            default_value="false",
            description="if true, the AR tag mount is included in the 'robot_description' parameter; if false, it is left out; set to true if using the AR tag mount in your project",
        ),
        DeclareLaunchArgument(
            "show_gripper_bar",
            default_value="true",
            description="if true, the gripper_bar link is included in the 'robot_description' parameter; if false, the gripper_bar and finger links are not loaded to the parameter server. Set to false if you have a custom gripper attachment",
        ),
        DeclareLaunchArgument(
            "show_gripper_fingers",
            default_value="true",
            description="if true, the gripper fingers are included in the 'robot_description' parameter; if false, the gripper finger links are not loaded to the parameter server. Set to false if you have custom gripper fingers",
        ),
        DeclareLaunchArgument(
            "use_world_frame",
            default_value="true",
            description="set this to true if you would like to load a 'world' frame to the 'robot_description' parameter which is located exactly at the 'base_link' frame of the robot; if using multiple robots or if you would like to attach the 'base_link' frame of the robot to a different frame, set this to false",
        ),
        DeclareLaunchArgument(
            "external_urdf_loc",
            default_value='""',
            description="the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file",
        ),
        DeclareLaunchArgument(
            "load_gazebo_configs",
            default_value="false",
            description="set this to true if Gazebo is being used; it makes sure to include Gazebo related configs in the 'robot_description' parameter so that the robot models show up black in Gazebo",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="launches Rviz",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="launches Rviz",
        ),
        DeclareLaunchArgument(
            "model",
            default_value='""',
            description="file path to the robot-specific URDF including arguments to be passed in",
        ),
        DeclareLaunchArgument(
            "dof",
            default_value='5',
            description="the degrees of freedom of the arm; while the majority of the arms have 5 dof, others have 4 or 6 dof",
        ),
        DeclareLaunchArgument(
            "launch_control",
            default_value='true',
            description="brings up control launch",
        ),
        OpaqueFunction(function = launch_setup)
        ])