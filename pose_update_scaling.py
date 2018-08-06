#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from geometry_msgs.msg import Quaternion, Polygon, PolygonStamped,Point32
import tf
from math import cos, sin
import numpy as np
DEBUG=False
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
  e *= 0.05 #0.01
  std = np.sqrt(e)
  #print(std)
  dx = BASE_LENGTH/2 + std[0] * N_SIGMA
  dy = BASE_WIDTH/2 + std[1] * N_SIGMA
  dth = std[2] * N_SIGMA 
  quaternion = [0, 0, dth, 1.0]
  deuler = tf.transformations.euler_from_quaternion(quaternion)
  dyaw = deuler[2]
  dth = dyaw
  p = PolygonStamped()
  x = last_position.pose.position.x
  y = last_position.pose.position.y
  quaternion = [last_position.pose.orientation.x, last_position.pose.orientation.y, last_position.pose.orientation.z, last_position.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(quaternion)
  yaw = euler[2]
  th = yaw
  #print((x, dx))
  #print((y, dy))
  #print((th, dth))
  points = [
    Point32(x + (dx * cos(th-dth) - dy * sin(th-dth)),y + (dx * sin(th - dth) + dy*cos(th-dth)), 0),
    Point32(x + (dx * cos(th) - dy * sin(th)),y + (dx * sin(th) + dy*cos(th)), 0),
    Point32(x + (dx * cos(th+dth) - dy * sin(th+dth)),y + (dx * sin(th+ dth) + dy*cos(th+dth)), 0),
    
    Point32(x + (-dx * cos(th-dth) - dy * sin(th-dth)),y + (-dx * sin(th - dth) + dy*cos(th-dth)), 0),
    Point32(x + (-dx * cos(th) - dy * sin(th)),y + (-dx * sin(th)+ dy*cos(th)), 0),
    Point32(x + (-dx * cos(th+dth) - dy * sin(th+dth)),y + (-dx * sin(th + dth) + dy*cos(th+dth)), 0),

    Point32(x + (-dx * cos(th-dth) + dy * sin(th-dth)),y + (-dx * sin(th - dth) - dy*cos(th-dth)), 0),
    Point32(x + (-dx * cos(th)+ dy * sin(th)),y + (-dx * sin(th) - dy*cos(th)), 0),
    Point32(x + (-dx * cos(th+dth) + dy * sin(th+dth)),y + (-dx * sin(th + dth) - dy*cos(th+dth)), 0),

    Point32(x + (dx * cos(th-dth) + dy * sin(th-dth)),y + (dx * sin(th - dth) - dy*cos(th-dth)), 0),
    Point32(x + (dx * cos(th) + dy * sin(th)),y + (dx * sin(th) - dy*cos(th)), 0),
    Point32(x + (dx * cos(th+dth) + dy * sin(th+dth)),y + (dx * sin(th + dth) - dy*cos(th+dth)), 0)
    ]
  p.polygon.points = points
  p.header.stamp = rospy.Time.now()
  p.header.frame_id="map"
  ppub.publish(p)
  ip = PolygonStamped()
  ip.header.frame_id = "map"
  ip.header.stamp = rospy.Time.now()
  ip.polygon.points= [
          Point32(x + (BASE_LENGTH/2 * cos(th) - BASE_WIDTH/2 * sin(th)),y + (BASE_LENGTH/2 * sin(th) + BASE_WIDTH/2*cos(th)), 0),
          Point32(x + (-BASE_LENGTH/2 * cos(th) - BASE_WIDTH/2 * sin(th)),y + (-BASE_LENGTH/2 * sin(th)+ BASE_WIDTH/2*cos(th)), 0),
          Point32(x + (-BASE_LENGTH/2 * cos(th)+ BASE_WIDTH/2 * sin(th)),y + (-BASE_LENGTH/2 * sin(th) - BASE_WIDTH/2*cos(th)), 0),
          Point32(x + (BASE_LENGTH/2 * cos(th) + BASE_WIDTH/2 * sin(th)),y + (BASE_LENGTH/2 * sin(th) - BASE_WIDTH/2*cos(th)), 0)
              ]
  ipPub.publish(ip)

last_update = PoseWithCovariance()
last_position = PoseStamped()
rospy.init_node("dev_node")
if DEBUG:
    ppub = rospy.Publisher("/uncertain_position_dbg", PolygonStamped, queue_size=5)
else:
    ppub = rospy.Publisher("/uncertain_position", PolygonStamped, queue_size=5)
ipPub = rospy.Publisher("base_footprint", PolygonStamped, queue_size=3)
rospy.Subscriber("/poseupdate_fixed", PoseWithCovarianceStamped, cbp)
rospy.Subscriber("/slam_out_pose", PoseStamped, updatePose)
rospy.Timer(rospy.Duration(0.1), pcb)
rospy.spin()

