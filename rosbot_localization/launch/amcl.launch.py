from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="true"
    )

    map_name_arg = DeclareLaunchArgument(
        name="map_name", 
        default_value="small_house"
    )

    amcl_config_arg = DeclareLaunchArgument(
        name="amcl_config", 
        default_value=PathJoinSubstitution([
            FindPackageShare("rosbot_localization"), 
            "config", 
            "amcl.yaml"
        ])
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    map_path = PathJoinSubstitution([
        FindPackageShare("rosbot_mapping"), 
        "maps", 
        LaunchConfiguration("map_name"), 
        "map.yaml"
    ])

    lifecycle_nodes = ["map_server", "amcl"]

   

    nav2_map_server = Node(
        package="nav2_map_server", 
        executable="map_server",
        output="screen", 
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
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

    nav2_amcl = Node(
        package="nav2_amcl", 
        executable="amcl", 
        output="screen",
        parameters=[
            amcl_config, 
            {"use_sim_time": use_sim_time}
        ]
    )

    


    return LaunchDescription([
        use_sim_time_arg, 
        amcl_config_arg,
        map_name_arg,
        nav2_map_server, 
        nav2_lifecycle_manager, 
        nav2_amcl
    ])