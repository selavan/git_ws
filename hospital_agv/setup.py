from setuptools import setup

package_name = 'hospital_agv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    #add depedencies
    'setuptools',
    'rclpy',
    'std_msgs',
    ],
    zip_safe=True,
    maintainer='yok',
    maintainer_email='yok@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = hospital_agv.imu_publisher:main',
            'imu_publisher_t1 = hospital_agv.imu_publisher_t1:main',
            'imu_publisher_v1 = hospital_agv.imu_publisher_v1:main',
            'imu_sub_v1 = hospital_agv.imu_sub_v1:main',
            'odom_publisher = hospital_agv.odom_publisher:main',
            'robot_state_publisher = hospital_agv.robot_state_publisher:main',
            'server = hospital_agv.server:main',
            'send_goal = hospital_agv.send_goal:main',
            'tele_key = hospital_agv.tele_key:main',
            'twist_publisher = hospital_agv.twist_publisher:main',
            'key_twist_publisher = hospital_agv.key_twist_publisher:main',
            'key_twist_publisher_v1 = hospital_agv.key_twist_publisher_v1:main',
            'read_imu_arduino = hospital_agv.read_imu_arduino:main',
            'i2c_serial = hospital_agv.i2c_serial:main',
            'read_imu_arduino_v1 = hospital_agv.read_imu_arduino:main',
            'read_imu_arduino_v2 = hospital_agv.read_imu_arduino_v2:main',
            'read_imu_arduino_v3 = hospital_agv.read_imu_arduino_v3:main',
            'read_imu_arduino_v4_keep = hospital_agv.read_imu_arduino_v4_keep:main',
            'serial_read = hospital_agv.serial_read:main',
            'imu_plot = hospital_agv.imu_plot:main',
        ],
    },
)





















