from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config = FindPackageShare(package='hospital_agv', filename='rviz/wit901.rviz')

    imu_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    imu_publisher = Node(
        package='hospital_agv',  # Replace with the actual package name
        executable='imu_publisher_v1',  # Replace with the actual executable name
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        imu_broadcaster,
        imu_publisher,
        rviz,
        LogInfo(
            actions=[
                'Launch completed. Launch RViz and add IMU displays using the configuration in the rviz/imu.rviz file.'
            ]
        )
    ])

