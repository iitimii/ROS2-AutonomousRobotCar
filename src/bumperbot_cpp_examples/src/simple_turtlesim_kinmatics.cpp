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

   char log_buffer[256];
    int bytes_written = snprintf(log_buffer, sizeof(log_buffer),
                                 "\\n Translation Vector Turtle1 -> Turtle2\\n"
                                 "Tx: %.2f\\n"
                                 "Ty: %.2f\\n"
                                 "Theta: %.2f\\n",
                                 Tx, Ty, theta_deg);

    if (bytes_written > 0 && bytes_written < static_cast<int>(sizeof(log_buffer))) {
        RCLCPP_INFO(get_logger(), "%s", log_buffer);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to format log message");
}
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

