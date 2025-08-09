from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_description"),
            "launch", 
            "gazebo.launch.py"
        ])
    )

    controller = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_control"), 
            "launch", 
            "controller.launch.py"
        ])
    )

    joystick = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_control"), 
            "launch", 
            "joystick.launch.py"
        ]),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    display = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_description"),
            "launch", 
            "display.launch.py"
        ]),
        launch_arguments={
            "use_state_publisher": "False"
        }.items()
    )

    return LaunchDescription([
        gazebo,
        controller,
        display,
        joystick
    ])