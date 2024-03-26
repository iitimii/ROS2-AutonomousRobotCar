#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

using namespace std::chrono_literals;
using r_int = std_msgs::msg::Int32;

class ComplexPublisher : public rclcpp::Node
{
    private:
    rclcpp::Publisher<r_int> pub;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter;
    unsigned int multiple;

    void timer_callback(unsigned int multiple)
    {
        auto num = r_int();
        num.data = ++counter*multiple;
        pub.publish(num);
    }

    public:
    ComplexPublisher() : Node("Ticker")
    {
        pub = this->create_publisher<r_int>("Stream", 10);
        timer_ = create_wall_timer(1s);

    }
};