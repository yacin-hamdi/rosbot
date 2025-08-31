from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        name="use_slam", 
        default_value="true"
    )

    use_slam = LaunchConfiguration("use_slam")

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

    # display = IncludeLaunchDescription(
    #     PathJoinSubstitution([
    #         FindPackageShare("rosbot_description"),
    #         "launch", 
    #         "display.launch.py"
    #     ]),
    #     launch_arguments={
    #         "use_state_publisher": "False"
    #     }.items()
    # )

    rviz2_slam = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", 
        arguments=[
            "-d", 
            PathJoinSubstitution([
                FindPackageShare("rosbot_mapping"), 
                "rviz", 
                "slam.rviz"])
            ],
        condition=IfCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_mapping"), 
            "launch", 
            "slam.launch.py"
        ]), 
        condition=IfCondition(use_slam)
    )

    amcl = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("rosbot_localization"), 
            "launch", 
            "amcl.launch.py"
        ]), 
        condition=UnlessCondition(use_slam)
    )
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        rviz2_slam,
        # display,
        joystick, 
        slam,
        amcl
    ])