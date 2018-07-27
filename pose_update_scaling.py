#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from geometry_msgs.msg import Polygon, PolygonStamped,Point32
import numpy as np

BASE_LENGTH = 0.22
BASE_WIDTH = 0.25
N_SIGMA = 1.96
def cbp(pose):
  global last_update
  last_update = pose.pose
def updatePose(pose):
    global last_position
    last_position = pose

def pcb(event):
  cv = []
  for x in range(0,6):
    cv.append([])
    for y in range(0, 6):
      cv[x].append(last_update.covariance[6*x+y])
  cv = np.array(cv)
  e = np.linalg.eigvals(cv)
  e *= 0.01
  std = np.sqrt(e)
  print(std)
  dx = BASE_LENGTH/2 + std[0] * N_SIGMA
  dy = BASE_WIDTH/2 + std[1] * N_SIGMA
  p = PolygonStamped()
  points = [
    Point32(last_position.pose.position.x - dx, last_position.pose.position.y - dy, 0),
    Point32(last_position.pose.position.x - dx, last_position.pose.position.y + dy, 0),
    Point32(last_position.pose.position.x + dx, last_position.pose.position.y + dy, 0),
    Point32(last_position.pose.position.x + dx, last_position.pose.position.y - dy, 0)
    ]
  p.polygon.points = points
  p.header.stamp = rospy.Time.now()
  p.header.frame_id="map"
  ppub.publish(p)
  print(p)

last_update = PoseWithCovariance()
last_position = PoseStamped()
rospy.init_node("dev_node")
ppub = rospy.Publisher("/uncertain_position", PolygonStamped, queue_size=5)
rospy.Subscriber("/poseupdate_fixed", PoseWithCovarianceStamped, cbp)
rospy.Subscriber("/slam_out_pose", PoseStamped, updatePose)
rospy.Timer(rospy.Duration(0.1), pcb)
rospy.spin()

