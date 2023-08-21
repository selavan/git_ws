import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class ImuBridgeNode(Node):
    def __init__(self):
        super().__init__('imu_bridge_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_data', 10)

        # Open serial communication with Arduino
        self.serial_ = serial.Serial('/dev/ttyACM0', 115200)

        # Start reading data from serial
        self.timer_ = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_.in_waiting:
            data = self.serial_.readline().decode('latin-1').strip()  # Read a line from serial

            # Preprocess the data
            data = data.replace(' ', '')  # Remove spaces
            data = ''.join([c for c in data if c.isdigit() or c == '.'])  # Keep only digits and decimal point

            # Process the data and populate the message
            values = data.split(',')  # Assuming data is comma-separated values

            # Convert values to floats with error handling
            msg = Float32MultiArray()
            msg.data = []
            for value in values:
                try:
                    float_value = float(value)
                    msg.data.append(float_value)
                except ValueError:
                    # Handle the error here, such as skipping the value or using a default value
                    print(f"Warning: Failed to convert value '{value}' to float.")

            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_bridge_node = ImuBridgeNode()
    rclpy.spin(imu_bridge_node)
    imu_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

