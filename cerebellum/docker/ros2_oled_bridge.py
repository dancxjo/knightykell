#!/usr/bin/env python3
import os
import time
import socket
import json
import threading

import rclpy
from rclpy.node import Node

SOCK_PATH = os.environ.get('OLED_SOCK', '/run/oled/statusd.sock')


def oled_send(header, *lines):
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        s.connect(SOCK_PATH)
        s.send(json.dumps({'header': header, 'lines': list(lines)}).encode('utf-8'))
        s.close()
    except Exception:
        pass


class OLEDBridge(Node):
    def __init__(self):
        super().__init__('oled_bridge')
        self.timer = self.create_timer(5.0, self.tick)
        self.count = 0

    def tick(self):
        try:
            nodes = self.get_node_names_and_namespaces()
            topics = self.get_topic_names_and_types()
            ncount = len(nodes)
            tcount = len(topics)
            self.count += 1
            oled_send('ROS2', f'nodes: {ncount}', f'topics: {tcount}', f'beat: {self.count}')
        except Exception as e:
            self.get_logger().warn(f'bridge error: {e}')


def main():
    rclpy.init()
    node = OLEDBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

