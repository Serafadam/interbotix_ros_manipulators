from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_prefix = get_package_share_directory("interbotix_xsarm_descriptions")

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value='""',
            description="name of the robot (typically equal to robot_model, but could be anything)'",
        )
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            "rvizconfig",
            default_value='""',
            description="file path to the config file Rviz should load",
        )
    )

    robot_name = LaunchConfiguration("robot_name")
    rvizconfig = LaunchConfiguration("rvizconfig")

    rvizconfig = PathJoinSubstitution(
        [description_prefix, "rviz", "xsarm_description.rviz"]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=robot_name,
        name="rviz2",
        output="log",
        arguments=["-d", rvizconfig],
    )

    nodes_to_start = [
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
