#!/usr/bin/env python3
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # UDP Setup
        self.udp_ip = "0.0.0.0"
        self.udp_port = 3334
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.1)
        
        self.get_logger().info("Motor UDP Subscriber Started")

    def spin(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                try:
                    parts = data.decode().strip().split(',')
                    if len(parts) >= 2:  # Accepts "L,R" or "L,R,SEQ"
                        left = float(parts[0])
                        right = float(parts[1])
                        
                        twist = Twist()
                        # Normalize PWM [-255,255] to Twist [-1.0,1.0]
                        twist.linear.x = (left + right) / 2000.0  
                        twist.angular.z = (right - left) / 2000.0
                        self.publisher.publish(twist)
                        
                        if len(parts) >= 3:
                            self.get_logger().debug(f"Seq: {parts[2]}")
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f"Invalid data: {data.decode()}")
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Fatal: {str(e)}")
                break


def main(args=None):
    rclpy.init(args=args)
    node = MotorSubscriber()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()