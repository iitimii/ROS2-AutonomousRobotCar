#include <bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp>

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string& node_name) : Node(node_name)
{
    turtle1_pose_sub = create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1));
    turtle2_pose_sub = create_subscription<turtlesim::msg::Pose>("turtle2/pose", 10, std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1));
}