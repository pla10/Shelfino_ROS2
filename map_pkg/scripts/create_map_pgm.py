#!/usr/bin/env python3

import os, sys
import cv2
import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Polygon
from obstacles_msgs.msg import ObstacleArrayMsg
from geometry_msgs.msg import Point32

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
        self.resolution = 0.005
        self.borders_done = False
        self.obstacles_done  = False
        self.borders_sub = self.create_subscription(Polygon,"map_borders",self.listen_borders,qos_profile)
        self.obstacles_sub = None
        self.share_dir = get_package_share_directory("shelfino_navigation")
        self.get_logger().info("Create map node started!")
        

    def __create_map(self, msg):
        self.max_x = np.max([point.x for point in msg.points])
        self.min_x = np.min([point.x for point in msg.points])
        self.max_x = math.ceil(self.max_x) if self.max_x > 0 else math.floor(self.max_x)
        self.min_x = math.ceil(self.min_x) if self.min_x > 0 else math.floor(self.min_x)
        self.max_y = np.max([point.y for point in msg.points])
        self.min_y = np.min([point.y for point in msg.points])
        self.max_y = math.ceil(self.max_y) if self.max_y > 0 else math.floor(self.max_y)
        self.min_y = math.ceil(self.min_y) if self.min_y > 0 else math.floor(self.min_y)

        self.img = np.zeros((int((self.max_x-self.min_x+self.padding)/self.resolution),int((self.max_y-self.min_y+self.padding)/self.resolution)))
        
        pts = np.array([self.__comp_point_wrt_map(point) for point in msg.points])
        isClosed = True
        color = (255,255,255)
        thickness = 20
        
        cv2.polylines(self.img, [pts], isClosed, color, thickness)



    def listen_borders(self, msg):
        if not self.borders_done:
            self.get_logger().info("Receiving borders...")
            self.borders_done = True
            self.get_logger().info("Number of points: '%d'" % len(msg.points))

            if len(msg.points) != 4 and len(msg.points) != 6:
                raise Exception(f"Invalid number of points {len(msg.points)} for the map borders")
            
            self.get_logger().info(f"Creating map with {len(msg.points)} points...")
            
            self.__create_map(msg)

            cv2.imwrite(os.path.join(self.share_dir, "maps", "dynamic_map.png"), self.img)
            cv2.imwrite(os.path.join(self.share_dir, "maps", "dynamic_map.pgm"), self.img)
            self.get_logger().info("Map created!")

            self.obstacles_sub = self.create_subscription(ObstacleArrayMsg,"obstacles",self.listen_obstacles,qos_profile)

    def __comp_point_wrt_map(self, point):
        return [int((self.max_y-point.y+self.padding/2)/self.resolution), int((self.max_x-point.x+self.padding/2)/self.resolution)]

    def listen_obstacles(self, msg):
        if not self.obstacles_done:
            self.get_logger().info("Receiving obstacles...")
            self.obstacles_done = True

            for obs in msg.obstacles:
                # This is a cylinder
                if len(obs.polygon.points) == 1:
                    radius = obs.radius
                    center = obs.polygon.points[0]
                    pts_tmp = [[center.x + radius * np.cos(theta), center.y + radius * np.sin(theta)] for theta in np.linspace(0, 2*np.pi, 360)]
                    pts = np.array([[self.__comp_point_wrt_map(Point32(x=point[0],y=point[1]))] for point in pts_tmp])
                else:
                    pts = np.array([self.__comp_point_wrt_map(point) for point in obs.polygon.points])
                
                color = (255,255,255)
                cv2.fillPoly(self.img, [pts],  color)

            self.img = np.rot90(self.img, k=-1, axes=(0, 1))
                
            cv2.imwrite(os.path.join(self.share_dir, "maps", "dynamic_map.png"), self.img)
            cv2.imwrite(os.path.join(self.share_dir, "maps", "dynamic_map.pgm"), self.img)
            self.get_logger().info("Map created!")

            with open(os.path.join(self.share_dir, "maps", "dynamic_map.yaml"), "w") as file:
                yaml_data = {'image' : 'dynamic_map.pgm',
                            'resolution' : self.resolution,
                            'origin': [-((self.max_x-self.min_x+self.padding)/2), -((self.max_y-self.min_y+self.padding)/2), 0.0],
                            'negate': 1,
                            'occupied_thresh': 0.65,
                            'free_thresh': 0.196}
                yaml.dump(yaml_data, file)

            self.get_logger().info(f"Files created! ({self.share_dir}/maps/dynamic_map)")
            sys.exit(0)

                

def main(args=None):
    # Delete the files
    share_dir = get_package_share_directory("shelfino_navigation")
    if os.path.exists(os.path.join(share_dir, "maps", "dynamic_map.png")):
        os.remove(os.path.join(share_dir, "maps", "dynamic_map.png"))
    if os.path.exists(os.path.join(share_dir, "maps", "dynamic_map.pgm")):
        os.remove(os.path.join(share_dir, "maps", "dynamic_map.pgm"))
    if os.path.exists(os.path.join(share_dir, "maps", "dynamic_map.yaml")):
        os.remove(os.path.join(share_dir, "maps", "dynamic_map.yaml"))

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