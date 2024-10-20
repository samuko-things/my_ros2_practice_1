#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts
 
 
class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")

        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)

        self.get_logger().info("add_two_ints_server node has started successfully")

    def callback_add_two_ints(self, req, res):
        res.sum = req.a + req.b
        self.get_logger().info("%ld + %ld = %ld" % (req.a, req.b, res.sum))
        return res
 
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()