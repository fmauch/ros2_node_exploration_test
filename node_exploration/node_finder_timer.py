#!/usr/bin/env python3

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

class NodeFinder(Node):
    def __init__(self):
        super().__init__('node_finder')
        self.node_name = '/talker'
        timer_period = 1.0
        self.found = False
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.found:
            return
        node_names_and_namespaces = self.get_node_names_and_namespaces()
        for entry in node_names_and_namespaces:
            name, namespace = entry
            fqn = namespace + ('' if namespace.endswith('/') else '/') + name
            if fqn == self.node_name:
                self.found = True
                raise SystemExit
        if self.found:
            self.get_logger().error(f'Did not find {self.node_name} node')


def main(args=None):
    rclpy.init(args=args)
    node_finder = NodeFinder()
    try:
        rclpy.spin(node_finder)
    except SystemExit:
        node_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
