import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped
from math import sin, cos
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.odom_broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.1
        self.vy = -0.1
        self.vth = 0.1 
        self.clock = self.get_clock()      
        self.current_time = self.clock.now()
        self.last_time = self.clock.now()

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        js = self._prepare_js()
        self.publisher_.publish(js)
    
    def _prepare_js(self):
        self.current_time = self.clock.now()

        dt = 1.0 * (self.current_time - self.last_time).nanoseconds/1e9
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th


        js = JointState()
        js.header.stamp = self.current_time.to_msg()
        js.header.frame_id = "odom"
        js.name = ['drivewhl_l_joint',]
        js.position = [3.0,]
        js.velocity = [0.1,]
        js.effort = [1.0,]
        return js
    
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
