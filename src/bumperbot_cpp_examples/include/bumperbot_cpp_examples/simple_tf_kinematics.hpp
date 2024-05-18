#ifndef SIMPLE_TF_KINEMATICS_HPP
#define SIMPLE_TF_KINEMATICS_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <bumperbot_msgs/srv/get_transform.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>

using GetTransform = bumperbot_msgs::srv::GetTransform;

class SimpleTfKinematics : public rclcpp::Node {
public:
    SimpleTfKinematics(const std::string& node_name);

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
    rclcpp::Service<GetTransform>::SharedPtr get_transform_srv_;

    geometry_msgs::msg::TransformStamped static_transform_stamped_;
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double x_increment_;
    double last_x_;

    int rotations_counter_;
    tf2::Quaternion orientation_increment_;
    tf2::Quaternion last_orientation_;


    void timer_callback_();
    bool get_transform_callback_(const std::shared_ptr<GetTransform::Request> req, const std::shared_ptr<GetTransform::Response> res);
};

#endif // SIMPLE_TF_KINEMATICS_HPP
