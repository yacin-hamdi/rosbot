from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="true"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    lifecycle_nodes = ["planner_server"]

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


    planner_server = Node(
        package="nav2_planner", 
        executable="planner_server", 
        name="planner_server", 
        output="screen", 
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rosbot_navigation"), 
                "config", 
                "planner_server.yaml"
            ]), 
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg, 
        planner_server, 
        nav2_lifecycle_manager
    ])