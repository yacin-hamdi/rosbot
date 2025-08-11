from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path



def generate_launch_description():
    rosbot_description = FindPackageShare("rosbot_description")
    

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=PathJoinSubstitution(
            [rosbot_description, "urdf", "rosbot.urdf.xacro"]
        )
    )
    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty.sdf"
    )

    model = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution([
        rosbot_description, 
        "worlds", 
        PythonExpression(["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])
    resource_path = str(Path(get_package_share_directory("rosbot_description")).parent.resolve())
    resource_path += os.pathsep + os.path.join(get_package_share_directory("rosbot_description"), "models")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value= resource_path
    )

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":robot_description, 
            "use_sim_time": True}
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
              FindPackageShare("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments=[("gz_args", PythonExpression(["' -v 4 -r ", world_path, "'"]))]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "rosbot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        # remappings=[
        #     ('/imu', '/imu/out')
        # ]
    )

    return LaunchDescription([
        model_arg, 
        world_name_arg,
        gazebo_resource_path, 
        robot_state_publisher, 
        gazebo, 
        gz_spawn_entity,
        gz_ros2_bridge
        
    ])


