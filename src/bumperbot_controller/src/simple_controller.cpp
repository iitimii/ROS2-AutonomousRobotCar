#include <bumperbot_controller/simple_controller.hpp>

using std::placeholders::_1;

WheelVelocityController::WheelVelocityController(const std::string &node_name) : Node(node_name)
{
    declare_parameter<double>("wheel_radius", 0.033);
    declare_parameter<double>("wheel_seperation", 0.16);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_seperation = get_parameter("wheel_seperation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Wheel radius: " << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Wheel seperation: " << wheel_seperation);

    cmd_vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/velocity", 10, std::bind(&WheelVelocityController::cmdVelCallback, this, _1));
    wheel_vel_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);

    wheels_to_velocity_matrix << wheel_radius / 2, wheel_radius / 2,
        wheel_radius / wheel_seperation, -wheel_radius / wheel_seperation;

    RCLCPP_INFO(get_logger(), "Simple velocity controller has been started.");
    RCLCPP_INFO_STREAM(get_logger(), "Wheels to velocity matrix: " << wheels_to_velocity_matrix);
}

void WheelVelocityController::cmdVelCallback(const geometry_msgs::msg::TwistStamped &cmd_vel)
{
    Eigen::Vector2d robot_speed(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
    Eigen::Vector2d wheel_velocities = wheels_to_velocity_matrix.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_velocities.coeff(1));
    wheel_speed_msg.data.push_back(wheel_velocities.coeff(0));

    wheel_vel_pub->publish(wheel_speed_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelVelocityController>("simple_velocity_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}