#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.clock import Clock

class MinimalPublisher(Node):

  def __init__(self):
    super().__init__('minimal_publisher')
    # self.publisher_ = self.create_publisher(String, 'topic', 10)
    self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    t = self.get_clock().now()
    print('t', t)
    goal.header.stamp = t.to_msg()
    goal.pose.position.x = 0.0
    goal.pose.position.y = -1.0
    goal.pose.position.z = 0.0
    self.publisher_.publish(goal)
    print(goal)



def main(args=None):
  print("Trying set a goal!")
  rclpy.init(args=args)
  pub = MinimalPublisher()
  rclpy.spin(pub)
  pub.destroy()
  rclpt.shutdown()


if __name__ == "__main__":
  main()

