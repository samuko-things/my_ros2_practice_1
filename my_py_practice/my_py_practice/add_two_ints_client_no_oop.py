#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")

    client_ = node.create_client(AddTwoInts, "add_two_ints")

    while not client_.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn("waithing for add_two_int_server ...")

    req = AddTwoInts.Request()
    req.a = 3
    req.b = 4

    future = client_.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        res = future.result()
        node.get_logger().info("%ld + %ld = %ld" % (req.a, req.b, res.sum))
    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))
    
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()