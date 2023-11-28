#!/usr/bin/env python3

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('node_finder')
    node_name = '/talker'
    timeout = node.get_clock().now() + Duration(seconds=5)
    found = False
    while node.get_clock().now() < timeout and not found:
        node_names_and_namespaces = node.get_node_names_and_namespaces()
        for entry in node_names_and_namespaces:
            name, namespace = entry
            fqn = namespace + ('' if namespace.endswith('/') else '/') + name
            if fqn == node_name:
                found = True
                break
        if not found:
            node.get_logger().error(f'Did not find {node_name} node')
            time.sleep(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
