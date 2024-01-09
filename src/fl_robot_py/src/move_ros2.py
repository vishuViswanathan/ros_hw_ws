import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from tutorial_interfaces.msg import Num
from std_msgs.msg import Float64
import time


class LRPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(Float64, 'topic', 10)
        self.publisher_left_ = self.create_publisher(Float64, '/left_wheel_controller/command', 10)
        self.publisher_right_ = self.create_publisher(Float64, '/right_wheel_controller/command', 10)

    def pub_cmd(selg, left, roght):
        self.publisher_left_.publish(left)
        self.publisher_right_.publish(right)
 

def main(args=None):
    rclpy.init(args=args)

    publisher = LRPublisher()

    rclpy.spin(publisher)
    print("Moving with py!")
    cmdl = 0
    cmdr = 0

    #command_pub.publish(cmd)
    time.sleep(1)

    cmdl = 2
    cmdr = 2
    publisher.pub_cmd(cmdl, cmdr)
    time.sleep(30)

    cmdl = 2
    cmdr = 4
    publisher.pub_cmd(cmdl, cmdr)
    time.sleep(10)


    cmdl = 0
    cmdr = 0
    publisher.pub_cmd(cmdl, cmdr)
    
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
