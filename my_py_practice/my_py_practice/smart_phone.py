#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
 
 
class SmartPhoneNode(Node):
    def __init__(self):
        super().__init__("smart_phone")

        self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("smart_phone node has started")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)
 

def main(args=None):
    rclpy.init(args=args)
    node = SmartPhoneNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()