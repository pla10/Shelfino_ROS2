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

def is_in_map_points(points1 : list, points2 : list):
    polygon1 = Polygon(points1)
    polygon2 = Polygon(points2)
    assert polygon1.is_valid and polygon2.is_valid
    return polygon2.contains(polygon1) and not polygon1.touches(polygon2)

def is_in_map_polygons(points1 : Polygon, points2 : Polygon):
    assert points1.is_valid and points2.is_valid
    return points2.contains(points1) and not points1.touches(points2)

def gen_hex_points(L, X = 0, Y = 0):
    sqrt3o2 = math.sqrt(3) / 2

    # X, Y are the center
    x0, y0 = X+L, Y
    x1, y1 = X+L/2, Y+L*sqrt3o2
    x2, y2 = X-L/2, Y+L*sqrt3o2
    x3, y3 = X-L, Y
    x4, y4 = X-L/2, Y-L*sqrt3o2
    x5, y5 = X+L/2, Y-L*sqrt3o2

    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

def gen_rect_points(L1, L2):
    return [(L1/2.0, L2/2.0), (-L1/2.0, L2/2.0), (-L1/2.0, -L2/2.0), (L1/2.0, -L2/2.0)]

def circle(X, Y, R):
    return Point(X, Y).buffer(R)

def square(X, Y, L, yaw=0):
    return rotate(Polygon([
        (X-L/2.0, Y-L/2.0),
        (X+L/2.0, Y-L/2.0),
        (X+L/2.0, Y+L/2.0),
        (X-L/2.0, Y+L/2.0)
    ]), math.degrees(yaw), origin='center')

def rectangle(X, Y, L1, L2, yaw=0):
    return rotate(Polygon([
        (X-L1/2.0, Y-L2/2.0),
        (X+L1/2.0, Y-L2/2.0),
        (X+L1/2.0, Y+L2/2.0),
        (X-L1/2.0, Y+L2/2.0)
    ]), math.degrees(yaw), origin='center')

def hexagon(X, Y, L, yaw=0):
    sqrt3o2 = math.sqrt(3) / 2

    [(x0, y0), (x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)] = gen_hex_points(L, X, Y)

    return rotate(Polygon([
        (x0, y0),
        (x1, y1),
        (x2, y2),
        (x3, y3),
        (x4, y4),
        (x5, y5)
    ]), math.degrees(yaw), origin='center')


def center (points):
    polygon = Polygon(points)
    assert polygon.is_valid
    return (centroid(polygon).coords[0][0], centroid(polygon).coords[0][1])