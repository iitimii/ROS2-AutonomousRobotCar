#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

using namespace std::chrono_literals;
using r_int = std_msgs::msg::Int32;

class ComplexPublisher : public rclcpp::Node
{
public:
    ComplexPublisher() : Node("Ticker")
    {
        pub = this->create_publisher<r_int>("Stream", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&ComplexPublisher::timer_callback, this));
        RCLCPP_INFO(get_logger(), "Started ticking in " + (multiple) + "s");
    }

private:
    rclcpp::Publisher<r_int>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter;
    unsigned int multiple{1};

    void timer_callback()
    {
        auto num = r_int();
        num.data = ++counter * multiple;
        pub->publish(num);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComplexPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}