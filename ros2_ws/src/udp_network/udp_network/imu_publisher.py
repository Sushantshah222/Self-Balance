#!/usr/bin/env python3
import socket
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        # UDP Setup
        self.udp_ip = "ESP32 IP"  # ESP32 IP
        self.udp_port = 3333
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info("IMU UDP Publisher Started")

    def imu_callback(self, msg:Imu):
        # Add coordinate transformation
        q = msg.orientation
        # Convert from ENU to NED frame (common for robotics)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = -math.copysign(math.pi / 2, sinp)
        else:
            pitch = -math.asin(sinp)
        data = f"{pitch:.4f},{roll:.4f},0"
        self.sock.sendto(data.encode(), (self.udp_ip, self.udp_port))

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()