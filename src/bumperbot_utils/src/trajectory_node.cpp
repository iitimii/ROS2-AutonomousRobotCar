#include <bumperbot_utils/trajectory_node.hpp>
using std::placeholders::_1;

TrajectoryPlotter::TrajectoryPlotter(const std::string& node_name) : Node(node_name)
{
    declare_parameter<std::string>("odom_topic", "/bumperbot_controller/odom");
    odom_topic_ = get_parameter("odom_topic").as_string();

    odom_sub_ = create_subscription<Odometry>(odom_topic_, 10, std::bind(&TrajectoryPlotter::odomCallback, this, _1));
    path_pub_ = create_publisher<Path>("/bumperbot_controller/trajectory", 10);

    bumperbot_path_.header.frame_id = "odom";
    bumperbot_path_.header.stamp = get_clock()->now();
}


void TrajectoryPlotter::odomCallback(const Odometry& msg)
{
    bumperbot_pose_.header = msg.header;
    bumperbot_pose_.pose = msg.pose.pose;

    bumperbot_path_.poses.push_back(bumperbot_pose_);
    bumperbot_path_.header.stamp = get_clock()->now();

    path_pub_->publish(bumperbot_path_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlotter>("trajectory_plotter");
    rclcpp::spin(node);
    rclcpp::shutdown();
}


/*
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <memory>
#include <vector>


using std::placeholders::_1;

class TrajectoryDrawer : public rclcpp::Node
{
public:
  TrajectoryDrawer() : Node("trajectory_drawer")
  {
    declare_parameter<std::string>("odom_topic", "bumperbot_controller/odom");
    std::string odom_topic = get_parameter("odom_topic").as_string();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&TrajectoryDrawer::odometryCallback, this, _1));
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("bumperbot_controller/trajectory", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  nav_msgs::msg::Path trajectory_;

  void odometryCallback(const nav_msgs::msg::Odometry &msg)
  {
    trajectory_.header.frame_id = msg.header.frame_id;
    geometry_msgs::msg::PoseStamped curr_pose;
    curr_pose.header.frame_id = msg.header.frame_id;
    curr_pose.header.stamp = msg.header.stamp;
    curr_pose.pose = msg.pose.pose;
    trajectory_.poses.push_back(curr_pose);

    trajectory_pub_->publish(trajectory_);
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryDrawer>());
  rclcpp::shutdown();
  return 0;
}*/