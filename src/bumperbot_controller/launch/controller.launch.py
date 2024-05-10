from launch import LaunchDescription
from launch_ros.actions import Node


joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
)

simpler_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['simple_velocity_controller', '--controller-manager', '/controller_manager'],
)

def generate_launch_description():
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simpler_controller_spawner,
    ])