import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

class ArduinoDataReader(Node):
    def __init__(self):
        super().__init__('arduino_data_reader')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/imu',
            self.imu_callback,
            10
        )

        self.subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            10
        )

    def imu_callback(self, msg):
        imu_data = msg.data
        orientation = imu_data.split('|')[0]
        angular_velocity = imu_data.split('|')[1]
        linear_acceleration = imu_data.split('|')[2]
        self.get_logger().info('Received IMU Data: Orientation={}, Angular Velocity={}, Linear Acceleration={}'.format(orientation, angular_velocity, linear_acceleration))

    def mag_callback(self, msg):
        mag_data = msg.data
        magnetic_field = mag_data.split('|')[0]
        self.get_logger().info('Received Magnetic Field Data: Magnetic Field={}'.format(magnetic_field))

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDataReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

