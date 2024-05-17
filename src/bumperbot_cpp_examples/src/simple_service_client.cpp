#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleServiceClient : public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b) : Node("simple_service_client")
    {
        client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");

        auto request = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }

private:
    rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO_STREAM(get_logger(), "Service Response: " << future.get()->sum);
        }

        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to get response from service.");
        }
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("simple_service_client"), "Usage: simple_service_client a b");
        return 1;
    }
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}