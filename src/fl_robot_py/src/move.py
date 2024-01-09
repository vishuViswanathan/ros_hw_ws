import rospy
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
command_pub = rospy.Publisher("motor_commands", String)

print("Moving with py!")
print("command_pub", command_pub)
rospy.init_node("move_with_py")

cmd = "STOP"

#command_pub.publish(cmd)
time.sleep(1)

cmd = "GO_REALLY_FAST"

command_pub.publish(cmd)
time.sleep(5)

cmd = "RIGHT"

command_pub.publish(cmd)
time.sleep(10)

cmd = "LEFT"

command_pub.publish(cmd)
time.sleep(10)

cmd = "STOP"

command_pub.publish(cmd)

