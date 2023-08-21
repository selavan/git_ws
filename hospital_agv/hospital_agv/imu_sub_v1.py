import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/imu',
            self.imu_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def imu_callback(self, msg):
        # Extract data from the IMU message and format it as a string
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        imu_str = f"Received IMU Message:\nOrientation: X={orientation.x}, Y={orientation.y}, Z={orientation.z}, W={orientation.w}\nAngular Velocity: X={angular_velocity.x}, Y={angular_velocity.y}, Z={angular_velocity.z}\nLinear Acceleration: X={linear_acceleration.x}, Y={linear_acceleration.y}, Z={linear_acceleration.z}"

        # Log the formatted string
        self.get_logger().info(imu_str)

def main(args=None):
    rclpy.init(args=args)
    node = IMUSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

