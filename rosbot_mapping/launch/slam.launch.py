from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        name="slam_config", 
        default_value=PathJoinSubstitution([
            FindPackageShare("rosbot_mapping"), 
            "config", 
            "slam_toolbox.yaml"
        ])
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    lifecycle_nodes = ["map_saver_server"]

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node", 
        name="slam_toolbox", 
        output="screen", 
        parameters=[
            slam_config, 
            {"use_sim_time": use_sim_time}
        ]
    )

    nav2_map_server = Node(
        package="nav2_map_server", 
        executable="map_saver_server", 
        name="map_saver_server", 
        output="screen", 
        parameters=[
            {"save_map_timeout": 5.0}, 
            {"use_sim_time": True}, 
            {"fresh_thresh_default": 0.196}, 
            {"occupied_thresh_default": 0.65}
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager", 
        executable="lifecycle_manager", 
        output="screen", 
        parameters=[
            {"node_names": lifecycle_nodes}, 
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )



    return LaunchDescription([
        use_sim_time_arg, 
        slam_config_arg, 
        slam_toolbox, 
        nav2_map_server, 
        nav2_lifecycle_manager
    ])