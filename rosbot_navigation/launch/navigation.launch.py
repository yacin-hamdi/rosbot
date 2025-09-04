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

    lifecycle_nodes = ["planner_server", "controller_server", "behavior_server", "bt_navigator"]

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

    controller_server = Node(
        package="nav2_controller", 
        executable="controller_server", 
        name="controller_server", 
        output="screen", 
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rosbot_navigation"),
                "config", 
                "controller_server.yaml"
            ]), 
            {"use_sim_time": use_sim_time}
        ]
    )

    bt_navigator = Node(
        package="nav2_bt_navigator", 
        executable="bt_navigator", 
        name="bt_navigator", 
        output="screen", 
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rosbot_navigation"), 
                "config", 
                "bt_navigator.yaml"
            ]), 
            {"use_sim_time": use_sim_time}
        ]
    )

    behavior_server= Node(
        package="nav2_behaviors", 
        executable="behavior_server",
        name="behavior_server",
        output="screen", 
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("rosbot_navigation"), 
                "config", 
                "behavior_server.yaml"
            ])
        ]
    )

    return LaunchDescription([
        use_sim_time_arg, 
        planner_server, 
        controller_server,
        behavior_server,
        bt_navigator,
        nav2_lifecycle_manager
    ])