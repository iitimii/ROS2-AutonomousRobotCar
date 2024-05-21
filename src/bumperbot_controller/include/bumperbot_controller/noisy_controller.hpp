#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using JointState = sensor_msgs::msg::JointState;
using Odometry = nav_msgs::msg::Odometry;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class NoisyController : public rclcpp::Node
{
private:
    rclcpp::Subscription<JointState>::SharedPtr joint_sub_;

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;


    double wheel_radius {};
    double wheel_seperation {};
    Eigen::Matrix2d wheels_to_velocity_matrix;

    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    rclcpp::Time prev_time_;

    double x_;
    double y_;
    double theta_;

    Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    TransformStamped transform_stamped_;

    void jointCallback(const JointState& msg);

public:
    NoisyController(const std::string &node_name);
};

#endif // SIMPLE_CONTROLLER_HPP