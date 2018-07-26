#!/usr/bin/env python
import rospy
import geometry_msgs
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
import numpy as np
from shapely.geometry import Polygon

def ros2shapelyPolygon(polygon):
    points = []
    for point32 in polygon.polygon.points:
        points.append((point32.x, point32.y))
    poly = Polygon(points)
    return poly
def ros2shapelyGrid(grid, x, y, size):
    tiles = []
    x_min = int(round((x-size) / grid.info.resolution))
    x_max = int(round((x+size) / grid.info.resolution))    
    y_min = int(round((y-size) / grid.info.resolution))
    y_max = int(round((y+size) / grid.info.resolution))
    width = grid.info.width
    height = grid.info.height
    res = grid.info.resolution
    for x in range(max(0, x_min), min(width, x_max)):
        for y in range(max(0, y_min), min(height, y_max)):
            if grid.data[x*width+y] >= 100:
                tiles.append(Polygon([(x * res, y* res), (x* res + res, y*res), (x*res+res, y*res+res), (x*res, y*res+res)]))
    return tiles
def checkCollision(footprint, tiles):
    for t in tiles:
        if footprint.intersects(t):
            return True
    return False

def cbPoly(polygon):
    p = ros2shapelyPolygon(polygon)
    global last_footprint
    last_footprint = p
    centre = p.centroid.coords[0]
    global last_position
    last_position = Point32(centre[0], centre[1],0)
def cbGrid(og):
    global restricted
    restricted = ros2shapelyGrid(grid, last_position.x, last_position.y, 1)
def check(event):
    if last_position is None or restricted is None:
        print("still waiting for footprint and restriction map to become available")
        return
    collision = False
    for p in restricted:
        if p.intersects(last_footprint):
            collision = True
    if collision:
        print("potential collision detected")
        print(last_footprint)
        print(restricted)
last_footprint = None
last_position = None
restricted = None
rospy.init_node("probabilistic_collision_checker")
rospy.Subscriber("/uncertain_position", geometry_msgs.msg.PolygonStamped, cbPoly)
rospy.Subscriber("/move_restrictions", OccupancyGrid, cbGrid)
rospy.Timer(rospy.Duration(1), check)
rospy.spin()
