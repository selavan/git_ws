import rclpy
from rclpy.node import Node
import serial

class SerialReceiver(Node):
    def __init__(self):
        super().__init__('serial_receiver')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)  # Replace '/dev/ttyUSB0' with your serial port

    def receive_data(self):
        while self.ser.is_open:
            if self.ser.in_waiting > 0:
                data = self.ser.read()
                self.get_logger().info(f'Received data: {data}')

def main(args=None):
    rclpy.init(args=args)
    serial_receiver = SerialReceiver()
    try:
        serial_receiver.receive_data()
    finally:
        serial_receiver.ser.close()
        serial_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

