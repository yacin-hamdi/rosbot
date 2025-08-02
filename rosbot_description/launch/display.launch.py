from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    rosbot_description_dir = get_package_share_directory("rosbot_description")
    robot_description_arg = DeclareLaunchArgument(
        name="robot_description", 
        default_value=os.path.join(rosbot_description_dir, "urdf", "rosbot.urdf.xacro")
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("robot_description")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        output="screen",
        parameters=[
            {"robot_description": robot_description}
            ]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", 
        executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", 
        arguments=["-d", os.path.join(rosbot_description_dir, "rviz", "display.rviz")]
    )






    return LaunchDescription([
        robot_description_arg, 
        robot_state_publisher, 
        joint_state_publisher_gui, 
        rviz2
    ])