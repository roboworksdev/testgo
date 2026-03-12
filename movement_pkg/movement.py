import rclpy
from rclpy.node import Node
from booster_interface.msg import LowState, LowCmd


class Movement(Node):
    def __init__(self):
        super().__init__('movement')

        # === Subscribers ===
        # self.subscription = self.create_subscription(
        #     LowState, '/low_state', self.low_state_callback, 10)

        # === Publishers ===
        # self.publisher = self.create_publisher(LowCmd, 'test_control', 10)

        # === Timers ===
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        pass

    # def low_state_callback(self, msg):
    #     self.get_logger().info(f'IMU rpy: {msg.imu_state.rpy}')


def main(args=None):
    rclpy.init(args=args)
    node = Movement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
