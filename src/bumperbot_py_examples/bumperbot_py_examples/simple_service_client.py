import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client_.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

        self.future_ = self.client_.call_async(self.req)
        self.future_.add_done_callback(self.future_callback)

    def future_callback(self, future):
        self.get_logger().info('Result of add_two_ints: %d' % future.result().sum)


def main():
    rclpy.init()

    if len(sys.argv) < 3:
        print('Usage: simple_service_client.py <a> <b>')
        return -1
    
    node = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()