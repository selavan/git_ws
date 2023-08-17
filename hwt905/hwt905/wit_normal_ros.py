


import struct
import rclpy #may not use
import time  #may not use
import math
import threading #may not use
import serial
import serial.tools.list_ports
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf2_ros.transformations import quaternion_from_euler

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)

        self.buff = bytearray()
        self.key = 0
        #self.flag = False
        self.angle_flag = False
        #self.calibuff = []
        self.angle_degree = [0, 0, 0]
        self.angular_velocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        #self.mag_offset = [0, 0, 0]
        #self.mag_range = [0, 0, 0]
        #self.version = 0
        self.longitude_imu = 0
        self.latitude_imu = 0
        self.altitude_imu = 0
        
        self.readreg = 0
        self.flag = False
        self.calibuff = []
        self.iapflag = False #move to  __nit__
        self.mag_offset = [0, 0, 0]
        self.mag_range = [0, 0, 0]
        self.version = 0
        self.recordflag = False #move to  __nit__
        self.baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

        self.imu_pub = self.create_publisher(Imu, 'wit/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'wit/mag', 10)
        self.location_pub = self.create_publisher(NavSatFix, 'wit/location', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, 'wit/cali', self.callback, 10)

        self.create_timer(1.0, self.serial_data_callback)

        self.ser = serial.Serial() #not use --> need to check
        self.python_version = platform.python_version()[0] #not use --> need to check

        self.find_ttyUSB()

    def find_ttyUSB(self):
        self.get_logger().info('default serial port is /dev/ttyUSB0, if multiple serial port devices are identified, modify the serial port corresponding to the imu in the launch file')
        posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
        self.get_logger().info('There are %d %s serial port devices connected to the current PC: %s', len(posts), 'USB', posts)

    def checkSum(self, list_data, check_data):
        return sum(list_data) & 0xff == check_data

    def hex_to_short(self, raw_data):
        return list(struct.unpack("hhhh", bytes(raw_data)))

    def hex_to_data(self, raw_data):
        return list(struct.unpack("i", bytes(raw_data)))

    def hex_to_altitude(self, raw_data):
        return list(struct.unpack("h", bytes(raw_data)))

    def handleSerialData(self, raw_data):
        if self.python_version == '2':
            self.buff[self.key] = ord(raw_data)
        if self.python_version == '3':
            self.buff[self.key] = ord(raw_data)

        self.key += 1
        if self.buff[0] != 0x55:
            self.key = 0
            return

        if self.key < 11:
            return
        else:
            data_buff = list(self.buff)  # Convert dictionary values to a list
            if self.buff[1] == 0x51:
                if self.checkSum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                else:
                    self.get_logger().info('0x51 Check failure')
                    
            # ... Similar processing for other cases
            elif self.buff[1] == 0x52:
                  if self.checkSum(data_buff[0:10], data_buff[10]):
                      self.angular_velocity = [self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
                  else:
                      self.get_logger().info('0x52 Check failure')

            elif self.buff[1] == 0x53:
                  if self.checkSum(data_buff[0:10], data_buff[10]):
                      self.temp = self.hex_to_short(data_buff[2:10])
                      self.angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]
                      self.version = temp[3]
                      self.angle_flag = True #may have to assign self.angle_flag = False
                  else:
                      self.get_logger().info('0x53 Check failure')
            
            elif self.buff[1] == 0x54:
                  if self.checkSum(data_buff[0:10], data_buff[10]): 
                      self.magnetometer = self.hex_to_short(data_buff[2:10])
                      if self.flag:
	            self.calibuff.append(magnetometer[0:2])
                  else:
                      self.get_logger().info('0x54 Check failure')
                      
            elif self.buff[1] == 0x57:
                  if self.checkSum(data_buff[0:10], data_buff[10]):
                      self.longitude_imu = (self.hex_to_data(data_buff[2:6])[0]  // 10000000.0 * 100 ) +  ((hex_to_data(data_buff[2:6])[0]  % 10000000) / 10000000.0)
                      self.latitude_imu = (self.hex_to_data(data_buff[6:10])[0]  // 10000000.0 * 100 ) +((hex_to_data(data_buff[6:10])[0] % 10000000) / 10000000.0)
                  else:
                      self.get_logger().info('0x57 Check failure')

            elif self.buff[1] == 0x58:
                  if self.checkSum(data_buff[0:10], data_buff[10]): 
                      self.altitude_imu = self.hex_to_altitude(data_buff[2:4])[0]  / 10.0
                
                  else:
                      self.get_logger().info('0x58 Check failure')
                      
            elif self.buff[1] == 0x5f:
                  if self.checkSum(data_buff[0:10], data_buff[10]):
                      self.readval = self.hex_to_short(data_buff[2:10])
                      if self.readreg == 0x0b:
	                self.mag_offset = readval
                      else:
	                self.mag_range = readval
	                
                      self.get_logger().info('readval')
                  else:
                      self.get_logger().info('0x5f Check failure')

            else:
                self.buff = bytearray()
                self.key = 0

            self.buff = bytearray()
            self.key = 0

            if self.angle_flag:
                stamp = self.get_clock().now().to_msg()

                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "base_link"

                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = "base_link"

                location_msg = NavSatFix()
                location_msg.header.stamp = stamp
                location_msg.header.frame_id = "base_link"

                angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
                qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

                imu_msg.orientation.x = qua[0]
                imu_msg.orientation.y = qua[1]
                imu_msg.orientation.z = qua[2]
                imu_msg.orientation.w = qua[3]

                imu_msg.angular_velocity.x = self.angular_velocity[0]
                imu_msg.angular_velocity.y = self.angular_velocity[1]
                imu_msg.angular_velocity.z = self.angular_velocity[2]

                imu_msg.linear_acceleration.x = self.acceleration[0]
                imu_msg.linear_acceleration.y = self.acceleration[1]
                imu_msg.linear_acceleration.z = self.acceleration[2]

                mag_msg.magnetic_field.x = magnetometer[0]
                mag_msg.magnetic_field.y = magnetometer[1]
                mag_msg.magnetic_field.z = magnetometer[2]

                self.imu_pub.publish(imu_msg)
                self.mag_pub.publish(mag_msg)
                self.location_pub.publish(location_msg)

                location_msg.longitude = self.longitude_imu
                location_msg.latitude = self.latitude_imu
                location_msg.altitude = self.altitude_imu

    def callback(self, msg):
        # Similar to the callback function in your original code
         '''
         self.readreg = 0
         self.flag = False
         self.calibuff = []
         self.iapflag = False #move to  __nit__
         self.mag_offset = [0, 0, 0]
         self.mag_range = [0, 0, 0]
         self.version = 0
         self.recordflag = False #move to  __nit__
         self.baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]
         '''
         
         unlock_imu_cmd = b'\xff\xaa\x69\x88\xb5'
         reset_magx_offset_cmd = b'\xff\xaa\x0b\x00\x00'
         reset_magy_offset_cmd = b'\xff\xaa\x0c\x00\x00'
         reset_magz_offset_cmd = b'\xff\xaa\x0d\x00\x00'
         enter_mag_cali_cmd = b'\xff\xaa\x01\x09\x00'
         exti_cali_cmd = b'\xff\xaa\x01\x00\x00'
         save_param_cmd = b'\xff\xaa\x00\x00\x00'
         read_mag_offset_cmd = b'\xff\xaa\x27\x0b\x00'
         read_mag_range_cmd = b'\xff\xaa\x27\x1c\x00'
         reboot_cmd = b'\xff\xaa\x00\xff\x00'
         reset_mag_param_cmd = b'\xff\xaa\x01\x07\x00'
         set_rsw_demo_cmd = b'\xff\xaa\x02\x1f\x00'  #output time acc gyro angle mag
         
         if "mag" in msg.data:            
             self.ser.write(unlock_imu_cmd) #check
             time.sleep(0.1)
             self.ser.write(reset_magx_offset_cmd)
             time.sleep(0.1)
             self.ser.write(reset_magy_offset_cmd)
             time.sleep(0.1)
             self.ser.write(reset_magz_offset_cmd)
             time.sleep(0.1)
             self.ser.write(enter_mag_cali_cmd)
             time.sleep(0.1)
            
             self.flag = True
             self.calibuff = []
             self.mag_offset = [0, 0, 0]
             ag_range = [500, 500, 500]
             
         elif "exti" in msg.data:
            self.flag = False
            self.ser.write(unlock_imu_cmd)
            time.sleep(0.1)
            self.ser.write(exti_cali_cmd)
            time.sleep(0.1)
            self.ser.write(save_param_cmd)
            time.sleep(1)
            readreg = 0x0b
            self.ser.write(read_mag_offset_cmd)
            time.sleep(1)
            readreg = 0x1c
            self.ser.write(read_mag_range_cmd)
            time.sleep(1)
            datalen = len(self.calibuff)
            #print('cali data {}'.format(datalen))
            self.get_logger().info('cali data {} %d', .format(datalen))
            r = list()
            if datalen > 0:
                for i in range(datalen):
                    tempx = ((self.calibuff[i][0] - self.mag_offset[0])*2/float(self.mag_range[0]))
                    tempy = ((self.calibuff[i][1] - self.mag_offset[1])*2/float(self.mag_range[1]))
                    temp = tempx*tempx+tempy*tempy-1
                    r.append(abs(temp))
                sumval = sum(r)
                r_n = float(sumval)/datalen
                if r_n < 0.05:
                    #print('magnetic field calibration results are very good')
                    self.get_logger().info('magnetic field calibration results are very good %d')
                elif r_n < 0.1:
                    #print('magnetic field calibration results are good')
                    self.get_logger().info('magnetic field calibration results are very good %d')
                else :
                    #print('magnetic field calibration results is bad, please try again')
                    self.get_logger().info('magnetic field calibration results are very good %d')
                    
         elif "version" in msg.data:
             self.get_logger().info('Sensor version is %d', version)
             
         elif "begin" in msg.data:
             record_thread = threading.Thread(target=self.record_thread)
             record_thread.start()
             
         elif "stop" in msg.data:
             self.recordflag = False
         
         elif "rate" in msg.data:
             ratelist = [0.2, 0.5, 1,2,5,10,20,50,100,125,200]
             try:
                 val = msg.data[4:]
                 rate = float(val)
                 for i in range(len(ratelist)):
                     if rate == ratelist[i]:
                         #print('chage {} rate'.format(rate))
                         self.get_logger().info('chage {} rate %d', .format(rate))
                         val = i + 1
                         cmd = bytearray(5)
                         cmd[0] = 0xff
                         cmd[1] = 0xaa
                         cmd[2] = 0x03
                         cmd[3] = val
                         cmd[4] = 0x00
                         self.ser.write(unlock_imu_cmd)
                         time.sleep(0.1)
                         self.ser.write(cmd)
             except Exception as e:
                 #print(e)
                 self.get_logger().info('e %d', e)
                 
         elif "baud" in data.data:
             try:
                 val = data.data[4:]
                 baud = float(val)
                 for i in range(len(baudlist)):
                     if baud == baudlist[i]:
                         val = i + 1
                         cmd = bytearray(5)
                         cmd[0] = 0xff
                         cmd[1] = 0xaa
                         cmd[2] = 0x04
                         cmd[3] = val
                         cmd[4] = 0x00
                         self.ser.write(unlock_imu_cmd)
                         time.sleep(0.1)
                         self.ser.write(cmd)
                         time.sleep(0.1)
                         self.ser.baudrate = baud
             except Exception as e:
                 #print(e)
                 self.get_logger().info('e %d', e)
                 
         elif "rsw" in data.data:
             self.ser.write(unlock_imu_cmd)
             time.sleep(0.1)
             self.ser.write(set_rsw_demo_cmd)
             time.sleep(0.1)
         

    def serial_data_callback(self):
        try:
            buff_count = self.ser.inWaiting()
            if buff_count > 0 and iapflag == 0:
                buff_data = self.ser.read(buff_count)
                for i in range(0, buff_count):
                    self.handleSerialData(buff_data[i])
        except Exception as e:
            self.get_logger().error('Exception: %s', str(e))
            self.get_logger().error('IMU loss of connection, poor contact, or broken wire')
            rclpy.shutdown()
            #exit(0)

    def run(self):
        try:
            port = self.get_parameter('port').get_parameter_value().string_value
            baudrate = self.get_parameter('baud').get_parameter_value().integer_value

            self.ser.port = port
            self.ser.baudrate = baudrate

            if self.ser.is_open:
                self.get_logger().info('Serial port is already open.')
            else:
                self.ser.open()
                if self.ser.is_open:
                    self.get_logger().info('Serial port opened successfully.')
                else:
                    self.get_logger().error('Failed to open the serial port.')
                    rclpy.shutdown()
                    #exit(0)

            #self.create_timer(1.0, self.serial_data_callback)
            rclpy.spin(self)

        except Exception as e:
            self.get_logger().error('Exception: %s', str(e))
            rclpy.shutdown()
            #exit(0)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    imu_node.run()
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


