import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Read the joystick commands and publish them to the /joy topic
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory('bumperbot_controller'), "config", "joy_config.yaml")]
    )

    # Read the /joy topic and publish the velocity commands to the /cmd_vel topic
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory('bumperbot_controller'), "config", "joy_teleop_config.yaml")]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])