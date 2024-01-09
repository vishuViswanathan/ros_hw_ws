#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from mv_fl_robot.msg import fl_control
bridge = CvBridge()

command= rospy.Publisher('/mv_fl_robot/fl_control', fl_control)

v_base = 0.5  # base velocity m/s

# limit is cover radius(0.15m) + 0.02m
r_limit = 0.15 + 0.2

# check locations are -90, -60, -30, 0, 30, 60, 90 deg with North of the cover
# with 1440 rays of the laser from -pi to +pi of cover north, 30deg step is 120 and the point of interest is from 360 to 1080 in steps of 120
# point 360 is East
pts = np.arange(360, 1081, 120)

# turn Radius= k * a/2, where a is wheel distance
# turn R depending on sector of the cover within limit
# k +ve is turn right and -ve is turn left
# K depending on the distance at test points less the limit
# pits index, k pairs
k_list = [-0.5, -0.3, -0.2, 0, 0.2, 0.3, 0.5]
in_callback = False
act = 0
cb = 0
collection = [-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0]

def callback(msg):
  global in_callback
  in_turn = False
  k = 1
  if not in_callback:
    in_callback = True
    collect_pts(msg.ranges)
    min_pos = r_limit
    min_loc = -1
    loc = 0
    for d in collection:
      if d < min_pos:
        min_pos = d
        min_loc = loc
      loc += 1
    if min_loc >= 0:
      k = k_list[min_loc]
      in_turn = True
    in_callback = False
  if in_turn:
    turn(k)
  else:
    move()

def collect_pts(ranges) :
  global collection
  for i in range(len(collection)):
    collection[i] = ranges[pts[i]] 
  
     
def pub_cmd(speed, k, by_angle) :
  ctl = fl_control()
  ctl.speed = speed
  ctl.turn_radius_k = k
  ctl.by_angle = by_angle
  print(ctl)
  command.publish(ctl) 
  

def turn(k):
  # print(cb, act, "in turn", k)
  if k == 0:
    del_v = 2 * v_base 
  else:  
    del_v = v_base / k 
  l = (v_base + del_v) /2
  r = (v_base - del_v) /2
  pub_cmd(l, r, 0)

def move():
  # print(cb, act, "in move()", v_base)
  pub_cmd(v_base, 0, 0)

def stop() :
  pub_cmd(0, 0, 0)
  
def main():    
  print("Hey Universe!")
  rospy.init_node("read_and_control_node")
  stop()
  rospy.sleep(1)
  sub = rospy.Subscriber('/mv_fl_robot/laser_scan/scan', LaserScan, callback)
  move()
  rospy.spin()
  
if __name__ == "__main__":
  main()

