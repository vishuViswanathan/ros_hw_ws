#! /usr/bin/env python

import rclpy
from sensor_msgs.msg import LaserScan
pts = (719,)

def callback(msg):
  for i in pts:
    print(i, msg.ranges[i])

rclpy.init()
sub = rospy.Subscriber('/mv_fl_robot/laser_scan/scan', LaserScan, callback)
rclpy.spin()


