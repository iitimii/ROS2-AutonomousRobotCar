#include <bumperbot_controller/simple_controller.hpp>

using std::placeholders::_1;

WheelVelocityController::WheelVelocityController(const std::string &node_name) : Node(node_name),
 left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0), x_(0.0), y_(0.0), theta_(0.0)
{
    declare_parameter<double>("wheel_radius", 0.033);
    declare_parameter<double>("wheel_seperation", 0.17);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_seperation = get_parameter("wheel_seperation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Wheel radius: " << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "Wheel seperation: " << wheel_seperation);

    prev_time_ = get_clock()->now();

    cmd_vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, std::bind(&WheelVelocityController::cmdVelCallback, this, _1));
    joint_sub_ = create_subscription<JointState>("/joint_states", 10, std::bind(&WheelVelocityController::jointCallback, this, _1));

    wheel_vel_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    odom_pub_ = create_publisher<Odometry>("/bumperbot_controller/odom", 10);

    wheels_to_velocity_matrix << wheel_radius / 2, wheel_radius / 2,
        wheel_radius / wheel_seperation, -wheel_radius / wheel_seperation;

    odom_msg_.header.frame_id = "odom"; // Fixed frame
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    RCLCPP_INFO(get_logger(), "Simple velocity controller cpp has been started.");
    RCLCPP_INFO_STREAM(get_logger(), "Wheels to velocity matrix: " << wheels_to_velocity_matrix);
}

void WheelVelocityController::cmdVelCallback(const geometry_msgs::msg::TwistStamped &cmd_vel)
{
    Eigen::Vector2d robot_speed(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
    Eigen::Vector2d wheel_velocities = wheels_to_velocity_matrix.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_velocities.coeff(1));
    wheel_speed_msg.data.push_back(wheel_velocities.coeff(0));

    wheel_vel_pub->publish(wheel_speed_msg);
}

void WheelVelocityController::jointCallback(const JointState &msg)
{
    double dp_left = msg.position.at(1) - left_wheel_prev_pos_;
    double dp_right = msg.position.at(0) - right_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    prev_time_ = msg_time;

    double phi_dot_left = dp_left / dt.seconds();
    double phi_dot_right = dp_right / dt.seconds();
    Eigen::Vector2d wheel_vel_vec(phi_dot_right, phi_dot_left);
    Eigen::Vector2d velocity_vec = wheels_to_velocity_matrix * wheel_vel_vec;

    double linear_vel = velocity_vec.coeff(0);
    double angular_vel = velocity_vec.coeff(1);

    double d_s = (wheel_radius/2) * (dp_right + dp_left);
    double d_theta = (wheel_radius / wheel_seperation) * (dp_right - dp_left);

    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();

    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;

    odom_msg_.twist.twist.linear.x = linear_vel;
    odom_msg_.twist.twist.angular.z = angular_vel;

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = get_clock()->now();


    odom_pub_->publish(odom_msg_);
    transform_broadcaster_->sendTransform(transform_stamped_);


    // RCLCPP_INFO_STREAM(get_logger(), "Linear: " << linear_vel << ", Angular: " << angular_vel);
    // RCLCPP_INFO_STREAM(get_logger(), "x: " << x_ << ", y: " << y_ << " theta:" << theta_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelVelocityController>("simple_velocity_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}