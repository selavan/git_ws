import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

class IMUReceiver(Node):
    def __init__(self):
        super().__init__('imu_receiver')
        self.publisher_imu = self.create_publisher(Imu, '/imu/imu', 10)
        self.publisher_mag = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)

    def parse_imu_data(self, data):
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = struct.unpack('fff', data[:12])
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = struct.unpack('fff', data[12:24])
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = struct.unpack('fff', data[24:36])
        return imu_msg

    def parse_mag_data(self, data):
        mag_msg = MagneticField()
        mag_msg.header.frame_id = 'imu'
        mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = struct.unpack('fff', data[:12])
        return mag_msg

    def receive_data(self):
        while True:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().strip()
                if line[0] == ord('O'):
                    imu_data = line[1:37]
                    imu_msg = self.parse_imu_data(imu_data)
                    self.publisher_imu.publish(imu_msg)
                elif line[0] == ord('M'):
                    mag_data = line[1:13]
                    mag_msg = self.parse_mag_data(mag_data)
                    self.publisher_mag.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_receiver = IMUReceiver()
    imu_receiver.receive_data()
    rclpy.spin(imu_receiver)
    imu_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

