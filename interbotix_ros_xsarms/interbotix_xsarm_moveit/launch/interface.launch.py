import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, OpaqueFunction

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
    # planning_context
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


    kinematics_yaml = load_yaml(
        "interbotix_xsarm_moveit", "config/kinematics.yaml"
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_tutorial",
        package="interbotix_xsarm_moveit",
        namespace="rx200",
        executable="moveit_client",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return [move_group_demo]
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
