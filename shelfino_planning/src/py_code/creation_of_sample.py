#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Creation_sample_node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("creation_sample_node") # MODIFY NAME
        self.marker_publisher_ = self.create_publisher() 

def main(args=None):
    rclpy.init(args=args)
    node = Creation_sample_node() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
