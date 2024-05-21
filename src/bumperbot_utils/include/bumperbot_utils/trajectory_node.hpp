#ifndef TRAJECTORY_NODE_HPP
#define TRAJECTORY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"


using Odometry = nav_msgs::msg::Odometry;
using Path  = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class TrajectoryPlotter : public rclcpp::Node
{

    private:
    Path bumperbot_path_;
    Odometry odom_msg_;
    PoseStamped bumperbot_pose_;

    std::string odom_topic_;

    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Path>::SharedPtr path_pub_;

    void odomCallback(const Odometry& msg);

    public:
    TrajectoryPlotter(const std::string& node_name);

};


#endif // !TRAJECTORY_NODE_HPP