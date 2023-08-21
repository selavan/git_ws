#V5_updated
import rclpy
from rclpy.node import Node
import serial
import json

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.linear_accel_x = 0.0
        self.linear_accel_y = 0.0
        self.linear_accel_z = 0.0

        self.magnetometer_x = 0.0
        self.magnetometer_y = 0.0
        self.magnetometer_z = 0.0

        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                received_data = self.ser.read_until(b'\n').decode('utf-8')
                try:
                    data_dict = json.loads(received_data)
                    self.process_data(data_dict)
                except json.JSONDecodeError:
                    self.get_logger().error("Error decoding JSON data.")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

    def process_data(self, data_dict):
        # Process the JSON data here based on the keys (e.g., "O", "G", "L", "M")
        if "O" in data_dict:
            orientation_data = data_dict["O"]
            self.orientation_x = orientation_data[0]
            self.orientation_y = orientation_data[1]
            self.orientation_z = orientation_data[2]
            self.get_logger().info(f"Orientation: X={self.orientation_x}, Y={self.orientation_y}, Z={self.orientation_z}")

        if "G" in data_dict:
            gyro_data = data_dict["G"]
            self.gyro_x = gyro_data[0]
            self.gyro_y = gyro_data[1]
            self.gyro_z = gyro_data[2]
            self.get_logger().info(f"Gyro: X={self.gyro_x}, Y={self.gyro_y}, Z={self.gyro_z}")

        if "L" in data_dict:
            linear_accel_data = data_dict["L"]
            self.linear_accel_x = linear_accel_data[0]
            self.linear_accel_y = linear_accel_data[1]
            self.linear_accel_z = linear_accel_data[2]
            self.get_logger().info(f"Linear Acceleration: X={self.linear_accel_x}, Y={self.linear_accel_y}, Z={self.linear_accel_z}")

        if "M" in data_dict:
            magnetometer_data = data_dict["M"]
            self.magnetometer_x = magnetometer_data[0]
            self.magnetometer_y = magnetometer_data[1]
            self.magnetometer_z = magnetometer_data[2]
            self.get_logger().info(f"Magnetometer: X={self.magnetometer_x}, Y={self.magnetometer_y}, Z={self.magnetometer_z}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
#V5 --> get good output 
import rclpy
from rclpy.node import Node
import serial
import json

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                received_data = self.ser.read_until(b'\n').decode('utf-8')
                try:
                    data_dict = json.loads(received_data)
                    self.process_data(data_dict)
                except json.JSONDecodeError:
                    self.get_logger().error("Error decoding JSON data.")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

    def process_data(self, data_dict):
        # Process the JSON data here based on the keys (e.g., "O", "G", "L", "M")
        if "O" in data_dict:
            orientation_data = data_dict["O"]
            self.get_logger().info(f"Orientation: X={orientation_data[0]}, Y={orientation_data[1]}, Z={orientation_data[2]}")

        if "G" in data_dict:
            gyro_data = data_dict["G"]
            self.get_logger().info(f"Gyro: X={gyro_data[0]}, Y={gyro_data[1]}, Z={gyro_data[2]}")

        if "L" in data_dict:
            linear_accel_data = data_dict["L"]
            self.get_logger().info(f"Linear Acceleration: X={linear_accel_data[0]}, Y={linear_accel_data[1]}, Z={linear_accel_data[2]}")

        if "M" in data_dict:
            magnetometer_data = data_dict["M"]
            self.get_logger().info(f"Magnetometer: X={magnetometer_data[0]}, Y={magnetometer_data[1]}, Z={magnetometer_data[2]}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
#V4
import rclpy
from rclpy.node import Node
import serial

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.data_buffers = {}
        self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                received_data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                lines = received_data.splitlines()

                for line in lines:
                    data_type, data_str = line.split(' ', 1)

                    if data_type in ['O', 'G', 'L', 'M']:  # Check if the data_type is valid
                        data_points = data_str.split(',')

                        if len(data_points) == 3:
                            try:
                                data_float = [float(data) for data in data_points]

                                if data_type not in self.data_buffers:
                                    self.data_buffers[data_type] = []

                                self.data_buffers[data_type] = data_float

                                # You can perform further processing based on the data type
                                # For example:
                                if data_type == 'O':
                                    x, y, z = data_float
                                    self.get_logger().info(f"Orientation: X={x}, Y={y}, Z={z}")
                                elif data_type == 'G':
                                    x, y, z = data_float
                                    self.get_logger().info(f"Gyro: X={x}, Y={y}, Z={z}")
                                # Add more cases for other data types

                            except ValueError:
                                self.get_logger().error("Error converting data to float.")
                        else:
                            self.get_logger().warn(f"Received incomplete data for {data_type}.")
                    else:
                        self.get_logger().warn(f"Unknown data type: {data_type}")

            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
#V3
import rclpy
from rclpy.node import Node
import serial

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.data_buffer = ''
        self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                received_data = self.ser.read(self.ser.in_waiting).decode('utf-8')
                self.data_buffer += received_data

                while '\n' in self.data_buffer:
                    line, self.data_buffer = self.data_buffer.split('\n', 1)
                    data_points = line.split(',')

                    if len(data_points) == 12:
                        try:
                            data_float = [float(data) for data in data_points]

                            # Now you can use these variables or process the data as needed
                            data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12 = data_float

                            self.get_logger().info(f"Data 1: {data1}")
                            self.get_logger().info(f"Data 2: {data2}")
                            # Add more log messages or data processing here for other data points
                        except ValueError:
                            self.get_logger().error("Error converting data to float.")
                    else:
                        self.get_logger().warn("Received incomplete data.")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
#V2
import rclpy
from rclpy.node import Node
import serial

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data_points = line.split(',')  # Split the data using the comma as delimiter
                
                if len(data_points) == 12:  # Check if we received all 12 data points
                    try:
                        # Convert each data point to a float number
                        data_float = [float(data) for data in data_points]

                        # Assign the data to individual variables or store them in a list or dictionary
                        data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12 = data_float

                        # Now you can use these variables or process the data as needed
                        self.get_logger().info(f"Data 1: {data1}")
                        self.get_logger().info(f"Data 2: {data2}")
                        # Add more log messages or data processing here for other data points
                    except ValueError:
                        self.get_logger().error("Error converting data to float.")
                else:
                    self.get_logger().warn("Received incomplete data.")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


'''
#V1
import rclpy
from rclpy.node import Node
import serial

class ArduinoSerialReader(Node):
    def __init__(self):
        super().__init__('arduino_serial_reader')
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"Serial port opened: {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port. Make sure it's correct and not already in use.")
            return

        self.create_timer(0.001, self.read_serial_data)

    def read_serial_data(self):
        if self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received data: {line}")
            except UnicodeDecodeError:
                self.get_logger().error("Error decoding received data.")
        else:
            self.get_logger().warn("Serial port is not open.")
              
def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
