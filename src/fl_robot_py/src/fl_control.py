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

wh_span = 0.174
half_wh_span = wh_span / 2
wheel_rad = 0.035

command_left= rospy.Publisher("/left_wheel_controller/command", Float64)
command_right= rospy.Publisher("/right_wheel_controller/command", Float64)

def callback(msg):
  speed = msg.speed # in m/s, +ve is fwd
  turn_radius_k = msg.turn_radius_k # in factor of wheel span
  by_angle = msg.by_angle # in rad +ve is right
  wheel_w = speed / wheel_rad
  turn_radius = turn_radius_k * half_wh_span
  left_w = 0
  right_w = 0
  for_time = 0
 
  right_turn = (by_angle > 0)
  
  if by_angle == 0:  # move stright
    left_w = wheel_w
    right_w = wheel_w
  else:
    if turn_radius_k == 0:
      left_w = wheel_w / 2
      right_w = -wheel_w / 2
      # STILL TO DO
    else:
      delta_w = wheel_w / turn_radius_k
      left_w = wheel_w + delta_w
      right_w = wheel_w - delta_w
      if speed != 0:
        for_time = abs(by_angle / (speed / turn_radius))
      print("wheel_w:", wheel_w, "delta_w:", delta_w)
  
  if not right_turn:
    left_w, right_w = right_w, left_w
  pub_cmd(left_w, right_w, for_time)
  print("left_w:", left_w, "right_w:", right_w, "for_time", for_time)
  
    
def pub_cmd(left, right, for_time) :
  command_left.publish(left)
  command_right.publish(right)
  
  
def main():    
  print("in fl_control!")
  rospy.init_node("fl_control_node")
  sub = rospy.Subscriber('/mv_fl_robot/fl_control', fl_control, callback)
  rospy.spin()
  
if __name__ == "__main__":
  main()

