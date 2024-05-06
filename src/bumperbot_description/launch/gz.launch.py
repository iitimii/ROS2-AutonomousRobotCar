import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Get package directories
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    bumperbot_description_dir = get_package_share_directory('bumperbot_description')
    bumperbot_description_prefix = get_package_prefix('bumperbot_description')

    model_path = os.path.join(bumperbot_description_dir, 'models')
    model_path += os.pathsep + os.path.join(bumperbot_description_prefix, 'share')

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    # Launch Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    # Launch Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]))}]
    )

    # Spawn the robot entity
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bumperbot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot
    ])