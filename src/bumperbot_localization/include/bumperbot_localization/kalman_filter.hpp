#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

using Odometry = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;

class KalmanFilter : public rclcpp::Node
{
    private:
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_; // wheel encoder readings
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_; // imu readings

    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;

    double mean_;
    double variance_;
    double imu_angular_z_;
    double last_angular_z_;
    double motion_;
    bool is_first_odom;

    Odometry kalman_odom_;

    double motion_variance_;
    double measurement_varienace_;

    void odomCallback(const Odometry& msg);
    void imuCallback(const Imu& msg);
    void measurementUpdate();
    void statePrediction();


    public:
    KalmanFilter(const std::string& node_name);

};

#endif // !KALMAN_FILTER_HPP