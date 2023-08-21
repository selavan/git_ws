import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt

class ImuDataPlotter(Node):
    def __init__(self):
        super().__init__('imu_data_plotter_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/imu',  
            self.imu_callback,
            10)
            
        self.subscription = self.create_subscription(
            MagneticField,
            '/imu/mag', 
            self.mag_callback,
            10)
            
        self.qx = []
        self.qy = []
        self.qz = []
        self.qw = []  
        self.angular_vel_x = []
        self.angular_vel_y = []
        self.angular_vel_z = [] 
        self.linear_accel_x = []
        self.linear_accel_y = []
        self.linear_accel_z = []
        self.magnetometer_x = []
        self.magnetometer_y = []
        self.magnetometer_z = []
        
        plt.ion()  # Turn on interactive mode

    def imu_callback(self, msg):
        self.qx.append(msg.orientation.x)
        self.qy.append(msg.orientation.y)
        self.qz.append(msg.orientation.z) 
        self.qw.append(msg.orientation.w)
        self.angular_vel_x.append(msg.angular_velocity.x)
        self.angular_vel_y.append(msg.angular_velocity.y)
        self.angular_vel_z.append(msg.angular_velocity.z)
        self.linear_accel_x.append(msg.linear_acceleration.x)
        self.linear_accel_y.append(msg.linear_acceleration.y)
        self.linear_accel_z.append(msg.linear_acceleration.z)
    
    def mag_callback(self, msg):
        self.magnetometer_x.append(msg.magnetic_field.x)
        self.magnetometer_y.append(msg.magnetic_field.y)
        self.magnetometer_z.append(msg.magnetic_field.z)
        
        plt.clf()  
        plt.subplot(4, 1, 1)
        plt.plot(self.qx, label='Quaternion X')
        plt.plot(self.qy, label='Quaternion Y')
        plt.plot(self.qz, label='Quaternion Z')
        plt.plot(self.qw, label='Quaternion W')
        plt.xlabel('Sample')
        plt.ylabel('Quaternion')
        plt.title('Quaternion')
        plt.legend()
        
        plt.subplot(4, 1, 2)
        plt.plot(self.angular_vel_x, label='Angular Vel X')
        plt.plot(self.angular_vel_y, label='Angular Vel Y')
        plt.plot(self.angular_vel_z, label='Angular Vel Z')
        plt.xlabel('Sample')
        plt.ylabel('Angular Velocity')
        plt.title('IMU Angular Velocity') 
        plt.legend()
        
        plt.subplot(4, 1, 3)
        plt.plot(self.linear_accel_x, label='Linear Acc X')
        plt.plot(self.linear_accel_y, label='Linear Acc Y')
        plt.plot(self.linear_accel_z, label='Linear Acc Z')
        plt.xlabel('Sample')
        plt.ylabel('Linear Acceleration ')
        plt.title('IMU Linear Acceleration ') 
        plt.legend()
        
        plt.subplot(4, 1, 4)
        plt.plot(self.magnetometer_x, label='Magnetometer X')
        plt.plot(self.magnetometer_y, label='Magnetometer Y')
        plt.plot(self.magnetometer_z, label='Magnetometer Z')
        plt.xlabel('Sample')
        plt.ylabel('Magnetometer')
        plt.title('Magnetometer')
        plt.legend()

        plt.tight_layout()
        plt.pause(1/100)

def main(args=None):
    rclpy.init(args=args)
    node = ImuDataPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

