import rclpy
from rclpy.node import Node


class Movement(Node):
    def __init__(self):
        super().__init__('movement')

        # === Publishers ===
        # self.publisher = self.create_publisher(String, '/topic', 10)

        # === Subscribers ===
        # self.subscription = self.create_subscription(
        #     String, '/topic', self.listener_callback, 10)

        # === Timers ===
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        pass

    # def listener_callback(self, msg):
    #     self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
