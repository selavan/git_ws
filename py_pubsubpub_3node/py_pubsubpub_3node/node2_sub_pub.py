import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16


class Node2(Node):

    def __init__(self):
        super().__init__('node2')
        self.subscription = self.create_subscription(
            Int16,
            'topic1',
            self.subscription_callback,
            10
        )
        self.publisher_ = self.create_publisher(Int16, 'topic2', 10)

    def subscription_callback(self, msg):
        self.get_logger().info('Node2 Received: "%s"' % msg.data)

        # Process the received message
        processed_msg = Int16()
        processed_msg.data = msg.data * 2

        self.publisher_.publish(processed_msg)
        self.get_logger().info('Node2 Publishing: "%s"' % processed_msg.data)


def main(args=None):
    rclpy.init(args=args)

    node2 = Node2()

    rclpy.spin(node2)

    node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

