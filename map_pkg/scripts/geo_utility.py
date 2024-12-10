#!/usr/bin/env python3

from shapely.geometry import Polygon, Point
from shapely.affinity import rotate
from shapely import centroid
import math

def touches(points1 : list, points2 : list):
    polygon1 = Polygon(points1)
    polygon2 = Polygon(points2)
    assert polygon1.is_valid and polygon2.is_valid 
    return polygon1.touches(polygon2)

def contains(points1 : list, points2 : list):
    polygon1 = Polygon(points1)
    polygon2 = Polygon(points2)
    assert polygon1.is_valid and polygon2.is_valid
    return polygon1.contains(polygon2)

def is_in_map_points(points : list, map_points : list):
    polygon = Polygon(points)
    map = Polygon(map_points)
    assert polygon.is_valid and map.is_valid
    return map.contains(polygon) and not polygon.touches(map)

def is_in_map_polygons(polygon : Polygon, map : Polygon):
    assert polygon.is_valid and map.is_valid
    return map.contains(polygon) and not polygon.touches(map)

def is_intersecting(polygon1 : Polygon, polygon2 : Polygon):
    assert polygon1.is_valid and polygon2.is_valid
    return polygon1.intersects(polygon2)

def square(X, Y, L, yaw=0) -> Polygon:
    return rotate(Polygon([
        (X-L/2, Y-L/2),
        (X+L/2, Y-L/2),
        (X+L/2, Y+L/2),
        (X-L/2, Y+L/2)
    ]), math.degrees(yaw), origin='center')


def rectangle(X, Y, L1, L2, yaw=0) -> Polygon:
    return rotate(Polygon([
        (X-L1/2, Y-L2/2),
        (X+L1/2, Y-L2/2),
        (X+L1/2, Y+L2/2),
        (X-L1/2, Y+L2/2)
    ]), math.degrees(yaw), origin='center')
    

def hexagon(X, Y, L, yaw=0) -> Polygon:
    sqrt3o2 = math.sqrt(3) / 2

    # X, Y are the center
    x0, y0 = X+L, Y
    x1, y1 = X+L/2, Y+L*sqrt3o2
    x2, y2 = X-L/2, Y+L*sqrt3o2
    x3, y3 = X-L, Y
    x4, y4 = X-L/2, Y-L*sqrt3o2
    x5, y5 = X+L/2, Y-L*sqrt3o2

    return rotate(Polygon([
        (x0, y0),
        (x1, y1),
        (x2, y2),
        (x3, y3),
        (x4, y4),
        (x5, y5)
    ]), math.degrees(yaw), origin='center')


def circle(X, Y, R) -> Polygon:
    return Point(X, Y).buffer(R)

def center (points):
    polygon = Polygon(points)
    assert polygon.is_valid
    return (centroid(polygon).coords[0][0], centroid(polygon).coords[0][1])