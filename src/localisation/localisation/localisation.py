#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class Localisation(Node):

    def __init__(self):
        super().__init__("localisation")

def main():
    rclpy.init()
    node = Localisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
