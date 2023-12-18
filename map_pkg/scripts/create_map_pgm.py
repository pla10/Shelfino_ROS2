#!/usr/bin/env python3

import sys
import cv2
import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Polygon
from obstacles_msgs.msg import ObstacleArrayMsg
from geometry_msgs.msg import PoseArray

from ament_index_python.packages import get_package_share_directory

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class CreateMap(Node):

    def __init__(self):
        super().__init__("create_map_pgm")
        self.padding = 5
        self.resolution = 0.05
        self.borders_done = False
        self.obstacles_done  = False
        self.borders_sub = self.create_subscription(Polygon,"map_borders",self.listen_borders,qos_profile)
        self.obstacles_sub = None
        self.share_dir = get_package_share_directory("shelfino_navigation")

    def listen_borders(self, msg):
        if not self.borders_done:
            self.get_logger().info("Receiving borders...")
            self.borders_done = True
            self.get_logger().info("Number of points: '%d'" % len(msg.points))
            self.max_x = np.max([point.x for point in msg.points])
            self.min_x = np.min([point.x for point in msg.points])
            self.max_x = math.ceil(self.max_x) if self.max_x > 0 else math.floor(self.max_x)
            self.min_x = math.ceil(self.min_x) if self.min_x > 0 else math.floor(self.min_x)
            self.max_y = np.max([point.y for point in msg.points])
            self.min_y = np.min([point.y for point in msg.points])
            self.max_y = math.ceil(self.max_y) if self.max_y > 0 else math.floor(self.max_y)
            self.min_y = math.ceil(self.min_y) if self.min_y > 0 else math.floor(self.min_y)

            self.img = np.zeros((int((self.max_x-self.min_x+self.padding)/self.resolution),int((self.max_y-self.min_y+self.padding)/self.resolution)))
            
            pts = np.array([[int((self.max_y-point.y+self.padding/2)/self.resolution), int((self.max_x-point.x+self.padding/2)/self.resolution)] for point in msg.points])
            isClosed = True
            color = (255,255,255)
            thickness = 2
            
            cv2.polylines(self.img, [pts], isClosed, color, thickness)

            self.obstacles_sub = self.create_subscription(ObstacleArrayMsg,"obstacles",self.listen_obstacles,qos_profile)

    def listen_obstacles(self, msg):
        if not self.obstacles_done:
            self.get_logger().info("Receiving obstacles...")
            self.obstacles_done = True

            for obs in msg.obstacles:
                pts = np.array([[int((self.max_y-point.y+self.padding/2)/self.resolution), int((self.max_x-point.x+self.padding/2)/self.resolution)] for point in obs.polygon.points])
                isClosed = True
                color = (255,255,255)
                thickness = 5

                cv2.fillPoly(self.img, [pts],  color)

            self.img = np.rot90(self.img, k=-1, axes=(0, 1))
                
            cv2.imwrite(self.share_dir+"/maps/dynamic_map.png", self.img)
            cv2.imwrite(self.share_dir+"/maps/dynamic_map.pgm", self.img)

            with open(self.share_dir+"/maps/dynamic_map.yaml", "w") as file:
                yaml_data = {'image' : 'dynamic_map.pgm',
                            'resolution' : self.resolution,
                            'origin': [-((self.max_x-self.min_x+self.padding)/2), -((self.max_y-self.min_y+self.padding)/2), 0.0],
                            'negate': 1,
                            'occupied_thresh': 0.65,
                            'free_thresh': 0.196}
                documents = yaml.dump(yaml_data, file)

            self.get_logger().info(f"Files created! ({self.share_dir}/maps/dynamic_map)")
            sys.exit(0)

                

def main(args=None):
    rclpy.init(args=args)

    create_map_pgm = CreateMap()

    rclpy.spin(create_map_pgm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    create_map_pgm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()