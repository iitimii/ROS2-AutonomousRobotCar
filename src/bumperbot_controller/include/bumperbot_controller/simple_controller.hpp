#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class WheelVelocityController : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub;
    
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped& cmd_vel);

public:
    WheelVelocityController() : Node("wheel_velocity_controller");
};