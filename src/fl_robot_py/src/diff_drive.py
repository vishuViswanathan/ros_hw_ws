#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist

class Cmd_vel_subscriber(Node):
  def __init__(self):
    super().__init__('cmd_vel_listener')
    # self.subscription = self.create_subscription(
    #    String,
    #    'topic',
    #    self.listener_callback,
    #    10)
    self.subscription = self.create_subscription(
        Twist,
        '/cmd_vel',
        self.listener_callback,
        10)
    self.subscription  # prevent unused variable warning
 

  def listener_callback(self, msg):
    velx = msg.linear.x
    angularz = msg.angular.z
    print('velx', velx, 'angularz', angularz)
  

def main(args=None):    
  rclpy.init(args=args)
  cm_vel_listener = Cmd_vel_subscriber()

  rclpy.spin(cm_vel_listener)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  cm_vel_listener.destroy_node()
  rclpy.shutdown()
 
if __name__ == "__main__":
  main()

