import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64

class LRPublisher(Node):
    def __init__(self):
        super().__init__('lr_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.command_left= Node.create_publisher(self, Float64, '/left_wheel_controller/command', 10)
        self.command_right= Node.create_publisher(self, Float64, '/right_wheel_controller/command', 10)

    def pub_cmd(self, left, right) :
        self.command_left.publish(left)
        self.command_right.publish(right)

def main(args=None):
    rclpy.init(args=args)
    p = LRPublisher()
    print("Moving with py!")
    print("command_left", p.command_left)

    cmdl = Float64()
    cmdr = Float64()

    #command_pub.publish(cmd)
    time.sleep(1)

    cmdl.data = 2.0
    cmdr.data = 2.0
    p.pub_cmd(cmdl, cmdr)
    time.sleep(30)

    cmdl = 2.0
    cmdr = 4.0
    p.pub_cmd(cmdl, cmdr)
    time.sleep(10)


    cmdl = 0.0
    cmdr = 0.0
    p.pub_cmd(cmdl, cmdr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
  
  








