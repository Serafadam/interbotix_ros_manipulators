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
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.substitutions import Command
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
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
    dof = LaunchConfiguration("dof")
    model = LaunchConfiguration("model")

    urdf_path = PathJoinSubstitution(
        [
            get_package_share_directory("interbotix_xsarm_descriptions"),
            "urdf",
            robot_model,
        ]
    )

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

    config_path = os.path.join(
        get_package_share_directory("interbotix_xsarm_moveit"), "config"
    )
    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            config_path,
            f"/srdf/{robot_model.perform(context)}.srdf.xacro" " " "robot_name:=",
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

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("interbotix_xsarm_moveit", "config/kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization \
                               default_planner_request_adapters/FixWorkspaceBounds \
                               default_planner_request_adapters/FixStartStateBounds \
                               default_planner_request_adapters/FixStartStateCollision \
                               default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "interbotix_xsarm_moveit", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml(
        "interbotix_xsarm_moveit",
        f"config/controllers/{dof.perform(context)}dof_controllers.yaml",
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    remappings = [("rx201/get_planning_scene", "/rx201/get_planning_scene")]

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=robot_name,
        parameters=[
            {
                "planning_scene_monitor_options": {
                    "robot_description": "robot_description",
                    "joint_state_topic": "/rx201/joint_states",
                }
            },
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        remappings=remappings
    )

    rviz_config_file = (
        get_package_share_directory("interbotix_xsarm_moveit") + "/launch/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        remappings=remappings
    )
    ros_control_prefix = os.path.join(
        get_package_share_directory("interbotix_xsarm_ros_control")
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [ros_control_prefix, "launch", "xsarm_ros_control.launch.py"]
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
            "use_world_frame": use_world_frame,
            "external_urdf_loc": external_urdf_loc,
            "use_rviz": "false",
            "use_sim": "true",
        }.items(),
    )

    return [
        run_move_group_node,
        rviz_node,
        # control
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
                "model",
                default_value='""',
                description="file path to the robot-specific URDF including arguments to be passed in",
            ),
            DeclareLaunchArgument(
                "dof",
                default_value="5",
                description="the degrees of freedom of the arm; while the majority of the arms have 5 dof, others have 4 or 6 dof",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )