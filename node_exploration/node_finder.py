#!/usr/bin/env python3

# Copyright 2023 Felix Exner <git@fexner.de>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Felix Exner <git@fexner.de> nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
