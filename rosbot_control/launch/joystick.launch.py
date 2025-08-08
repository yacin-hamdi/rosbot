from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    rosbot_control_dir = get_package_share_directory("rosbot_control")

    joy_teleop = Node(
        package="joy_teleop", 
        executable="joy_teleop", 
        parameters=[
            os.path.join(rosbot_control_dir, "config", "joy_teleop.yaml"), 
            {"use_sim_time": use_sim_time}
        ]
    )

    joy_node = Node(
        package="joy", 
        executable="joy_node", 
        parameters=[
            os.path.join(rosbot_control_dir, "config", "joy_config.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )



    return LaunchDescription([
        use_sim_time_arg, 
        joy_teleop, 
        joy_node
    ])  