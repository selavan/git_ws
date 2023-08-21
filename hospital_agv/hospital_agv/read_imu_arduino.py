
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_imu = self.create_publisher(Imu, 'imu_data', 10)
        self.publisher_mag = self.create_publisher(MagneticField, 'mag_data', 10)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate)

    def read_data(self):
        while rclpy.ok():
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode().strip()
                    self.process_data(line)
            except serial.SerialException as e:
                self.get_logger().error('Serial communication error: {}'.format(e))
            except Exception as e:
                self.get_logger().error('Error occurred: {}'.format(e))

    def process_data(self, line):
        readings = line.split('|')
        for reading in readings:
            if len(reading) < 2:
                continue

            mode = reading[0]
            data = reading[1:].split(',')
            if len(data) != 3:
                continue

            try:
                if mode == 'O':
                    self.publish_imu_data(data)
                elif mode == 'M':
                    self.publish_mag_data(data)
            except ValueError as e:
                self.get_logger().error('Error while parsing data: {}'.format(e))

    def publish_imu_data(self, data):
        msg = Imu()
        # Set the values of msg according to the parsed data
        # ...
        self.publisher_imu.publish(msg)

    def publish_mag_data(self, data):
        msg = MagneticField()
        # Set the values of msg according to the parsed data
        # ...
        self.publisher_mag.publish(msg)

    def shutdown(self):
        self.serial.close()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        arduino_reader.read_data()
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import threading

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_imu = self.create_publisher(Imu, 'imu_data', 10)
        self.publisher_mag = self.create_publisher(MagneticField, 'mag_data', 10)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate)
        self.thread = threading.Thread(target=self.read_data)
        self.thread.start()

    def read_data(self):
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode().strip()
                    readings = line.split('|')

                    for reading in readings:
                        if len(reading) < 2:
                            continue

                        mode = reading[0]
                        data = reading[1:].split(',')
                        if len(data) != 3:
                            continue

                        if mode == 'O':
                            self.publish_imu_data(data)
                        elif mode == 'M':
                             self.publish_mag_data(data)

                except ValueError as e:
                    self.get_logger().error('Error while parsing data: {}'.format(e))

    def publish_imu_data(self, data):
        msg = Imu()
        # Set the values of msg according to the parsed data
        # ...
        self.publisher_imu.publish(msg)

    def publish_mag_data(self, data):
        msg = MagneticField()
        # Set the values of msg according to the parsed data
        # ...
        self.publisher_mag.publish(msg)

    def shutdown(self):
        self.serial.close()
        self.thread.join()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        rclpy.spin(arduino_reader)
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(Float32, 'arduino_data', 10)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate)

    def read_data(self):
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    data = float(line)
                    msg = Float32()
                    msg.data = data
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().error('Invalid data: {}'.format(line))

    def shutdown(self):
        self.serial.close()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        arduino_reader.read_data()
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(Float32, 'arduino_data', 10)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate)

    def read_data(self):
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                try:
                    data = float(line)
                    msg = Float32()
                    msg.data = data
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().error('Invalid data: {}'.format(line))

    def shutdown(self):
        self.serial.close()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        arduino_reader.read_data()
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 9600
        self.serial = serial.Serial(self.serial_port, self.baud_rate)
        self.thread = threading.Thread(target=self.read_data)
        self.thread.start()

    def read_data(self):
        while rclpy.ok():
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)

    def shutdown(self):
        self.serial.close()
        self.thread.join()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        rclpy.spin(arduino_reader)
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.timer_ = self.create_timer(0.1, self.publish_data)

        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial = serial.Serial(self.serial_port, self.baud_rate)

    def publish_data(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode().strip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)

    def shutdown(self):
        self.serial.close()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    try:
        rclpy.spin(arduino_reader)
    except KeyboardInterrupt:
        pass
    arduino_reader.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
