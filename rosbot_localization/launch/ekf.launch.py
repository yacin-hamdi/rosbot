from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare("rosbot_localization"),
        "config", 
        "ekf.yaml"
    ])

    static_transform_publisher = Node(
        package="tf2_ros", 
        executable="static_transform_publisher", 
        arguments=["--x", "0", "--y", "0", "--z", "0.103", 
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", 
                   "--frame-id", "base_footprint_ekf", 
                   "--child-frame-id", "imu_link"]
    )

    ekf_node = Node(
        package="robot_localization", 
        executable="ekf_node", 
        name="ekf_node", 
        output="screen",
        parameters=[
            ekf_config
        ]
    )


    return LaunchDescription([
        static_transform_publisher, 
        ekf_node
    ])