#ifndef SIMPLE_TURTLESIM_KINEMATICS_HPP
#define SIMPLE_TURTLESIM_KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SimpleTurtlesimKinematics : public rclcpp::Node
{
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub;

    turtlesim::msg::Pose last_turtle1_pose;
    turtlesim::msg::Pose last_turtle2_pose;

    void turtle1PoseCallback(const turtlesim::msg::Pose& pose);
    void turtle2PoseCallback(const turtlesim::msg::Pose& pose);

    public:
    SimpleTurtlesimKinematics(const std::string& node_name);
}

#endif