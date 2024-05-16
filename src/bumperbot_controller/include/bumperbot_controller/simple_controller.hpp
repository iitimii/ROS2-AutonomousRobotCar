#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


class WheelVelocityController : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub;

    double wheel_radius {};
    double wheel_seperation {};
    Eigen::Matrix2d wheels_to_velocity_matrix;

    
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped& cmd_vel);

public:
    WheelVelocityController(const std::string &node_name);
};

#endif // SIMPLE_CONTROLLER_HPP