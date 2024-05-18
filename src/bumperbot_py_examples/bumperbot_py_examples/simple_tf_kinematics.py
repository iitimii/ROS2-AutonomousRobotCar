import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from bumperbot_msgs.srv import GetTransform

class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_tf_msg = TransformStamped()
        self.dynamic_tf_msg = TransformStamped()

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.static_tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_msg.header.frame_id = "bumperbot_base"
        self.static_tf_msg.child_frame_id = "bumperbot_top"

        self.static_tf_msg.transform.translation.x = 0.0
        self.static_tf_msg.transform.translation.y = 0.0
        self.static_tf_msg.transform.translation.z = 0.3

        self.static_tf_msg.transform.rotation.x = 0.0
        self.static_tf_msg.transform.rotation.y = 0.0
        self.static_tf_msg.transform.rotation.z = 0.0
        self.static_tf_msg.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(self.static_tf_msg)

        self.get_logger().info("SimpleTFKinematics Node Started")
        self.get_logger().info(
            f"Published static transform from {self.static_tf_msg.header.frame_id} to {self.static_tf_msg.child_frame_id}"
        )

        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.x_increment = 0.05
        self.last_x = 0.0

        self.get_transform_srv = self.create_service(
            GetTransform, "get_transform", self.get_transform_callback
        )

    def timer_callback(self):
        self.dynamic_tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_msg.header.frame_id = "odom"
        self.dynamic_tf_msg.child_frame_id = "bumperbot_base"

        self.dynamic_tf_msg.transform.translation.x = self.last_x + self.x_increment
        self.dynamic_tf_msg.transform.translation.y = 0.0
        self.dynamic_tf_msg.transform.translation.z = 0.0

        self.dynamic_tf_msg.transform.rotation.x = 0.0
        self.dynamic_tf_msg.transform.rotation.y = 0.0
        self.dynamic_tf_msg.transform.rotation.z = 0.0
        self.dynamic_tf_msg.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_tf_msg)

        self.last_x = self.dynamic_tf_msg.transform.translation.x

    def get_transform_callback(self, req, res):
        self.get_logger().info(f"Requesting transform from {req.frame_id} to {req.child_frame_id}")
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())

        except TransformException as e:
            self.get_logger().error(f"Failed to get transform from {req.frame_id} to {req.child_frame_id}: {e}")
            res.success = False
            return res

        res.transform = requested_transform
        res.success = True

        return res 


def main(args=None):
    rclpy.init(args=args)

    simple_tf_kinematics = SimpleTFKinematics()

    rclpy.spin(simple_tf_kinematics)

    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
