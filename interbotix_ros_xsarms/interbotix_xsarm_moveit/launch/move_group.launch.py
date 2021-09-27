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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
import yaml


# Reference: https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes/
# run_move_group/launch/run_move_group.launch.py

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
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
            "true",
            " ",
            "show_gripper_fingers:=",
            "true",
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

    config_path = os.path.join(
        get_package_share_directory('interbotix_xsarm_moveit'),
        'config')
    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            config_path,
            "/srdf/rx200.srdf.xacro"
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


    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('interbotix_xsarm_moveit', 'config/kinematics.yaml')

    # Planning Functionality
    ompl_planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization \
                               default_planner_request_adapters/FixWorkspaceBounds \
                               default_planner_request_adapters/FixStartStateBounds \
                               default_planner_request_adapters/FixStartStateCollision \
                               default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml('interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('interbotix_xsarm_moveit', 'config/controllers/5dof_controllers.yaml')

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.1}

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               namespace='rx200',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters])

    rviz_config_file = get_package_share_directory(
        'interbotix_xsarm_moveit') + '/launch/moveit.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     namespace='rx200',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml])


    return LaunchDescription([
                              run_move_group_node,
                              rviz_node,
                              ])