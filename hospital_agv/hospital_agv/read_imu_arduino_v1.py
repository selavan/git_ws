import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class HumbleSubscriber(Node):
    def __init__(self):
        super().__init__('humble_subscriber')
        self.subscription = self.create_subscription(
            Vector3,
            'orientation',
            self.orientation_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def orientation_callback(self, msg):
        self.get_logger().info('Received orientation: x={}, y={}, z={}'.format(
            msg.x, msg.y, msg.z
        ))

def main(args=None):
    rclpy.init(args=args)
    humble_subscriber = HumbleSubscriber()
    rclpy.spin(humble_subscriber)
    humble_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

