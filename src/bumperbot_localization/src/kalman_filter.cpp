#include "bumperbot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string& node_name) : Node(node_name),
mean_(0.0), variance_(1000.0),
imu_angular_z_(0.0), last_angular_z_(0.0), motion_(0.0),
is_first_odom(true),
motion_variance_(4.0), measurement_varienace_(0.5)
{
    odom_sub_ = create_subscription<Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, _1));
    imu_sub_ = create_subscription<Imu>("imu/out", 1000, std::bind(&KalmanFilter::imuCallback, this, _1));

    odom_pub_ = create_publisher<Odometry>("bumperbot_controller/odom_kalman", 10);

}

void KalmanFilter::odomCallback(const Odometry& msg)
{
    kalman_odom_ = msg;
    if (is_first_odom)
    {
        mean_ = msg.twist.twist.angular.z;
        last_angular_z_ = msg.twist.twist.angular.z;
        is_first_odom = false;
        return;
    }

    motion_ = msg.twist.twist.angular.z - last_angular_z_;

    statePrediction();
    measurementUpdate();

    kalman_odom_.twist.twist.angular.z = mean_;
    odom_pub_->publish(kalman_odom_);

}

void KalmanFilter::imuCallback(const Imu& msg)
{
    imu_angular_z_ = msg.angular_velocity.z;

}

void KalmanFilter::measurementUpdate()
{
    mean_ = (measurement_varienace_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_varienace_);
    variance_ = (variance_ * measurement_varienace_) / (variance_ + measurement_varienace_);
}

void KalmanFilter::statePrediction()
{
    mean_ = mean_ + motion_;
    variance_ = variance_ + motion_variance_;
}

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}