import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts


class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')

        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)

        self.get_logger().info("Simple Service Server has been started.")

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(f"New Request Received a: {request.a}, b: {request.b}")
        response.sum = request.a + request.b
        self.get_logger().info(f"Returning sum: {response.sum}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()