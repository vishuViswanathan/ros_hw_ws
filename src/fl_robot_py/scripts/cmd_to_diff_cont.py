import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelSubscriber(Node):

    def __init__(self, pub):
        super().__init__('cmd_vel_subscriber')
        # self.subscription = self.create_subscription(
        #    String,
        #    '/cmd_vel',
        #    self.listener_callback,
        #    10)
        self.pub = pub
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: ' )
        self.pub.publish(msg)


class CmdToDiffCOntPublisher(Node):

    def __init__(self):

        super().__init__('cmd_to_diff_cont_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    def publish(self, msg):
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pub_to_diff_cont = CmdToDiffCOntPublisher()
    cmd_vel_subscriber = CmdVelSubscriber(pub_to_diff_cont)
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    pub_to_diff_cont.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()