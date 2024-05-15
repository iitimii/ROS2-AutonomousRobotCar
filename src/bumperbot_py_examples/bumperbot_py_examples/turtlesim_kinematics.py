import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math


class TurtlesimKinematicsNode(Node):
    def __init__(self):
        super().__init__('turtlesim_kinematics_node')

        self.turtle1_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        self.turtle2_pose_sub_ = self.create_subscription(Pose, '/turtle2/pose', self.turtle2_pose_callback, 10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1_pose_callback(self, msg):
        self.last_turtle1_pose_ = msg

    def turtle2_pose_callback(self, msg):
        self.last_turtle2_pose_ = msg

        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = theta_rad * 180 / 3.14159265359


        self.get_logger().info("""\n
                               Translation vector turtle1 -> turtle2:\n
                               Tx: %f\n
                               Ty: %f\n
                               Rotation matrix turtle1 -> turtle2:\n
                               theta(rad): %f\n
                               theta(deg): %f\n
                               |R11   R12| : |%f   %f|\n
                               |R21   R22| : |%f   %f|\n """ % (Tx, Ty,  theta_rad, theta_deg,
                                                                math.cos(theta_rad), -math.sin(theta_rad),
                                                                math.sin(theta_rad), math.cos(theta_rad)))
        

def main():
    rclpy.init()
    turtlesim_kinematics_node = TurtlesimKinematicsNode()
    rclpy.spin(turtlesim_kinematics_node)
    turtlesim_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()