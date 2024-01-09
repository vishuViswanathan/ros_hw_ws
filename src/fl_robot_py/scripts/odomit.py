import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped
from math import sin, cos
from nav_msgs.msg import Odometry
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
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
        odom = self._prepare_odom()
        self.publisher_.publish(odom)
    
    def _prepare_odom(self):
        self.current_time = self.clock.now()

        dt = 1.0 * (self.current_time - self.last_time).nanoseconds/1e9
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = quaternion_from_euler(0, 0, self.th)
        # print('odom_quat', odom_quat)
        # print('odom_quat type', type(odom_quat))
#        self.odom_broadcaster.sendTransform(odom)   
#            (self.x, self.y, 0.),
#            odom_quat,
#            self.current_time,
#            "base_link",
#            "odom"
#        )

        # next, we'll publish the odometry message over ROS
        # set the position
        pt = Point()
        pt.x = self.x
        pt.z = 0.
        q = Quaternion()
        q.x, q.y, q.z, q.w  = odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]
        pose = Pose()
        pose.position = pt
        pose.orientation = q

        # set the velocity
        t = Twist()
        t.linear.x = self.vx
        t.linear.x = self.vy
        t.linear.z = 0.

        t.angular.x = 0.
        t.angular.y = 0.
        t.angular.z = self.vth

        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose # Pose(pt, q)
        odom.twist.twist = t

        tr = TransformStamped()
        tr.header.stamp = odom.header.stamp
        tr.header.frame_id = "base_link"
        tr.child_frame_id = "xxx"
        tr.transform.translation.x = self.x
        tr.transform.translation.y = self.y
        tr.transform.translation.z = 0.0

        tr.transform.rotation.x = q.x
        tr.transform.rotation.y = q.y
        tr.transform.rotation.z = q.z
        tr.transform.rotation.w = q.w

        self.odom_broadcaster.sendTransform( tr) 
        return odom
    
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
    
