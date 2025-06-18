from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # IMU Publisher
        Node(
            package='udp_network',
            executable='imu_publisher',
            name='IMUr',
            output='screen'
        ),

        # Motor Subscriber
        Node(
            package="udp_network",
            executable="motor_subscriber",
            name='Motor',
            output="screen",
        ),
    ])