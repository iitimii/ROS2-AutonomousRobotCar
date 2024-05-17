#!/Users/timii/micromamba/envs/ros2_env/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_seperation", 0.17)

        self.wheel_radius = (
            self.get_parameter("wheel_radius").get_parameter_value().double_value
        )
        self.wheel_seperation = (
            self.get_parameter("wheel_seperation").get_parameter_value().double_value
        )

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius}")
        self.get_logger().info(f"Using wheel seperation: {self.wheel_seperation}")
        self.get_logger().info("Simple Controller Node Py has been started")

        self.wheel_cmd_pub = self.create_publisher(
            "simple_velocity_controller/commands", Float64MultiArray, 10
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped, "bumperbot_controller/cmd_vel", self.velocity_callback, 10
        )

        self.wheels_to_local_velocity_matrix = np.array(
            [
                [self.wheel_radius / 2, self.wheel_radius / 2],
                [
                    self.wheel_radius / self.wheel_seperation,
                    -self.wheel_radius / self.wheel_seperation,
                ],
            ]
        )

    def velocity_callback(self, msg):
        robot_speed = np.array([msg.twist.linear.x, msg.twist.angular.z])
        wheel_speed = np.matmul(np.linalg.inv(self.wheels_to_local_velocity_matrix), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)


def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()