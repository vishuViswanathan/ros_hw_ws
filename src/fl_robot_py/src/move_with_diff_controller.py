import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from tutorial_interfaces.msg import Num
from std_msgs.msg import Float64
import time
from geometry_msgs.msg import Twist

class LRPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(Float64, 'topic', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def pub_cmd(self, msg):
        self.publisher.publish(msg)
  

def main(args=None):
    rclpy.init(args=args)
    print("Moving with py getting publisher!")
    publisher = LRPublisher()
    cmd = Twist()

#    rclpy.spin(publisher)
    print("Moving with py!")

    publisher.pub_cmd(cmd)
    time.sleep(1)

    cmd.linear.x = 0.2
    publisher.pub_cmd(cmd)
    time.sleep(5)

 
    cmd.linear.x = 0.0

    publisher.pub_cmd(cmd)
    
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
