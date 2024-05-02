import os
from os import pathsep

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")
    bumperbot_description_prefix = get_package_prefix('bumperbot_description')

    model_path = os.path.join(bumperbot_description, 'models')
    model_path += pathsep + os.path.join(bumperbot_description_prefix, 'share')

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), "launch", "gzserver.launch.py"
    )))
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), "launch", "gzclient.launch.py"
    )))

    spawn_robot = Node(
        package='ros_gz',
        executable='spawn_entity.py',
        arguments=['-entity', 'bumperbot', '-topic', 'robot_description'],
        output='screen'
    )
    

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])