#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import time
from geometry_msgs.msg import Twist

v_base = 0.5  # base velocity m/s


# turn Radius= k * a/2, where a is wheel distance
# turn R depending on sector of the cover within limit
# k +ve is turn right and -ve is turn left
# K depending on the distance at test points less the limit
# pits index, k pairs
in_callback = False
act = 0
cb = 0

class LRPublisher(Node):
  def __init__(self):
    super().__init__('cmd_vel_publisher')
    self.pub = self.create_publisher(Twist, '/cmd_vel', 5)
    self.vel_cmd = Twist()
    
  def pub_cmd(self, speed, k, by_angle) :
    self.vel_cmd.linear.x = speed
    self.vel_cmd.angular.z = k
    
    self.pub.publish(self.vel_cmd) 
    

class LidarSubscriber(Node):
  def __init__(self):
    super().__init__('lidar_listener')
    # self.subscription = self.create_subscription(
    #    String,
    #    'topic',
    #    self.listener_callback,
    #    10)
    self.subscription = self.create_subscription(
        LaserScan,
        '/scan',
        self.listener_callback,
        10)
    self.subscription  # prevent unused variable warning
    
    # check locations are -90, -60, -30, 0, 30, 60, 90 deg with North of the cover
    # with 180 rays of the laser from -pi;2 to +pi/2 of cover north, 30deg step is 120 
    # and the point of interest is from 0 to 180 in steps of 30
    # point 0 is East
    self.pts = np.arange(20, 161, 20)

    self.v_base = 0.5  # base velocity m/s

    # limit is cover radius(0.15m) + 0.02m
    self.r_limit = 0.15 + 0.2

    # turn Radius= k * a/2, where a is wheel distance
    # turn R depending on sector of the cover within limit
    # k +ve is turn right and -ve is turn left
    # K depending on the distance at test points less the limit
    # pits index, k pairs
#   self.k_list = [-0.5, -0.3, -0.2, 0.0, 0.2, 0.3, 0.5]
    self.k_list = [0.6, 1.0, 1.6, 1.0, -1.6, -1.0, -0.6]
    self.collection = [-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0]
    self.drive_controller = LRPublisher()

  def listener_callback(self, msg):
    global in_callback
    in_turn = False
    k = 1
    if not in_callback:
      in_callback = True
      self.collect_pts(msg.ranges)
      min_pos = self.r_limit
      min_loc = -1
      loc = 0
      for d in self.collection:
        if d < min_pos:
          min_pos = d
          min_loc = loc
        loc += 1
      if min_loc >= 0:
        k = self.k_list[min_loc]
        in_turn = True
      in_callback = False
    if in_turn:
      self.turn(k)
    else:
      self.move()

  def collect_pts(self,ranges) :
    for i in range(len(self.collection)):
      self.collection[i] = ranges[self.pts[i]] 
  
  def turn(self,k):
    # print(cb, act, "in turn", k)
    if k == 0:
      del_v = 2 * self.v_base 
    else:  
      del_v = self.v_base / k 
    l = (self.v_base + del_v) /2
    r = (self.v_base - del_v) /2
#    self.drive_controller.pub_cmd(l, r, 0)
    self.drive_controller.pub_cmd(0.0, k, 0)

  def move(self):
    # print(cb, act, "in move()", v_base)
    self.drive_controller.pub_cmd(self.v_base, 0.0, 0)

  def stop(self) :
    self.drive_controller.pub_cmd(0.0, 0.0, 0)

  
def main(args=None):    
  rclpy.init(args=args)
  lidar_listener = LidarSubscriber()

  rclpy.spin(lidar_listener)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  lidar_listener.destroy_node()
  rclpy.shutdown()
 
if __name__ == "__main__":
  main()

