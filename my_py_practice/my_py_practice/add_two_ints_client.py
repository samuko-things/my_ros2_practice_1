#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)
        self.call_add_two_ints_server(10, 9)
        self.call_add_two_ints_server(67, 13)

    def call_add_two_ints_server(self, req_a, req_b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waithing for add_two_int_server ...")

        req = AddTwoInts.Request()
        req.a = req_a
        req.b = req_b

        future = client.call_async(req)
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, req_a=req.a, req_b=req.b))

    def callback_call_add_two_ints(self, future, req_a, req_b):
        try:
            res = future.result()
            self.get_logger().info("%ld + %ld = %ld" % (req_a, req_b, res.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()