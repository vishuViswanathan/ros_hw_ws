! /usr/bin/env python

import rclpy
import numpy as np
from sensor_msgs.msg import LaserScan
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from mv_interfaces import Rcontrol

print (fl_control)

def main(args=None):
    rclpy.init(args=args)
    print('Rcontrol = ", Rcontrol) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
