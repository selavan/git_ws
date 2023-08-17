import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16


class Node3(Node):

    def __init__(self):
        super().__init__('node3')
        self.subscription = self.create_subscription(
            Int16,
            '/imu/imu',
            self.subscription_callback,
            10
        )

    def subscription_callback(self, msg):
        self.get_logger().info('Node3 Received: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node3 = Node3()

    rclpy.spin(node3)

    node3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

