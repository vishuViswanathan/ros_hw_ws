import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

pts = (60,)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
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
        # LOGGING TRIAL
        num = 4
        self.get_logger().info(f'My log message {num}', once=True) 
           
    def listener_callback(self, msg):
      for i in pts:
        print(i, msg.ranges[i])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
