from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    use_python_arg = DeclareLaunchArgument("use_python", default_value="false", description="Use Python controller")
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.033", description="Wheel Radius")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.17",description="Wheel separation")
    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="true")

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bumperbot_controller', '--controller-manager', '/controller_manager'],
        condition=UnlessCondition(use_simple_controller)
    )


    simpler_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller', '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_simple_controller)
    )

    ## This works also
    # wheel_vel_controller = Node(
    #     package="bumperbot_controller",
    #     executable="simple_controller.py" if LaunchConfiguration("use_python") == "true" else "simple_controller",
    #     parameters=[{
    #         "wheel_radius" : ParameterValue(LaunchConfiguration("wheel_radius")),
    #         "wheel_separation" : ParameterValue(LaunchConfiguration("wheel_separation"))
    #     }]
    # )

    wheel_vel_controller_py = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        parameters=[{
            "wheel_radius" : ParameterValue(wheel_radius),
            "wheel_separation" : ParameterValue(wheel_separation)
        }],
        condition=IfCondition(use_python)
    )

    wheel_vel_controller_cpp = Node(
        package="bumperbot_controller",
        executable="simple_controller",
        parameters=[{
            "wheel_radius" : ParameterValue(wheel_radius),
            "wheel_separation" : ParameterValue(wheel_separation)
        }],
        condition=UnlessCondition(use_python)
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
        simpler_controller_spawner,
        wheel_vel_controller_py,
        wheel_vel_controller_cpp,
        # wheel_vel_controller
    ])

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller
    ])