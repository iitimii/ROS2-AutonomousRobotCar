#include <bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp>

using std::placeholders::_1;

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &node_name) : Node(node_name)
{
    turtle1_pose_sub = create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));
    turtle2_pose_sub = create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}

void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle1_pose = pose;
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle2_pose = pose;
    float Tx = last_turtle1_pose.x - last_turtle2_pose.x;
    float Ty = last_turtle1_pose.y - last_turtle2_pose.y;
    float theta_rad = last_turtle1_pose.theta - last_turtle2_pose.theta;
    float theta_deg = theta_rad * 180 / M_PI;

        RCLCPP_INFO_STREAM(get_logger(), "Translation Vector Turtle1 -> Turtle2" << std::endl
            << "Tx: " << Tx << std::endl
            << "Ty: " << Ty << std::endl
            << "Theta: " << theta_deg << std::endl);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

