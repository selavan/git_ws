import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16


class Node1(Node):

    def __init__(self):
        super().__init__('node1')
        self.publisher_ = self.create_publisher(Int16, 'topic1', 10)
        timer_period = 0.01  # 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int16()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Node1 Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node1 = Node1()

    rclpy.spin(node1)

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





























