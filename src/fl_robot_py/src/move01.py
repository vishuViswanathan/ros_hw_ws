import rospy
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
command_left= rospy.Publisher("/left_wheel_controller/command", Float64)
command_right= rospy.Publisher("/right_wheel_controller/command", Float64)


def pub_cmd(left, right) :
  command_left.publish(left)
  command_right.publish(right)
  
  
print("Moving with py!")
print("command_left", command_left)
rospy.init_node("move_with_py")

cmdl = 0
cmdr = 0

#command_pub.publish(cmd)
time.sleep(1)

cmdl = 2
cmdr = 2
pub_cmd(cmdl, cmdr)
time.sleep(30)

cmdl = 2
cmdr = 4
pub_cmd(cmdl, cmdr)
time.sleep(10)


cmdl = 0
cmdr = 0
pub_cmd(cmdl, cmdr)




