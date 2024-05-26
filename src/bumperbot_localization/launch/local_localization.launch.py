import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_launch_description():

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf.yaml")]
    )

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher"
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher
    ])