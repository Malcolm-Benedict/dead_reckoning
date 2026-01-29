import rclpy
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

class DeadReckoner(Node):
    def __init__(self):
        super().__init__('estimator_node')
        self.dead_reckoning_path_publisher = self.create_publisher(Path, '/dead_reckoning/path', 10)
        self.dead_reckoning_odom_publisher = self.create_publisher(Odometry, '/dead_reckoning/odom', 10)

        #self.dead_reckoning_odom_publisher = self.create_publisher(Odometry, '/dead_reckoning/odom', 10)

        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_t = 0.0
        self.cmd_vel_v = 0.0
        self.cmd_vel_w = 0.0
        self.deltaT = 0.0
        self.last_timestamp = 0.0

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, qos_profile)

    def cmd_vel_callback(self,msg):
        #calculate current state est.
        self.deltaT = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nsec) - self.last_timestamp
        self.cmd_vel_x = self.cmd_vel_x + (self.cmd_vel_v * m.cos(self.cmd_vel_t) * self.deltaT)
        self.cmd_vel_y = self.cmd_vel_y + (self.cmd_vel_v * m.sin(self.cmd_vel_t) * self.deltaT)
        self.cmd_vel_t = self.cmd_vel_t + (self.cmd_vel_w * self.deltaT)
        #update values for next timestamp
        self.cmd_vel_v = msg.twist.linear.x
        self.cmd_vel_w = msg.twist.angular.z
        self.last_timestamp = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nsec)
        #publish values
        odom = Odometry()
        odom.pose.pose.position.x = self.cmd_vel_x
        odom.pose.pose.position.y = self.cmd_vel_y
        odom.pose.pose.orientation.

        

def main(args=None):
    rclpy.init(args=args)
    estimator_node = DeadReckoner()
    rclpy.spin(estimator_node)

    estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()