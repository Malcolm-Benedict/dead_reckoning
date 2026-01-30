import rclpy
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped, Pose
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
        self.path_arr = []

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, qos_profile)

    def cmd_vel_callback(self,msg):
        #calculate current state est.
        self.deltaT = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec) - self.last_timestamp
        self.cmd_vel_x = self.cmd_vel_x + (self.cmd_vel_v * m.cos(self.cmd_vel_t) * self.deltaT)
        self.cmd_vel_y = self.cmd_vel_y + (self.cmd_vel_v * m.sin(self.cmd_vel_t) * self.deltaT)
        self.cmd_vel_t = self.cmd_vel_t + (self.cmd_vel_w * self.deltaT)
        #update values for next timestamp
        self.cmd_vel_v = msg.twist.linear.x
        self.cmd_vel_w = msg.twist.angular.z
        self.last_timestamp = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec)
        
        #generate pose message
        current_pose = Pose()

        current_pose.position.x = self.cmd_vel_x
        current_pose.position.y = self.cmd_vel_y

        w,x,y,z = convertEulerToQuaternion(0,0,self.cmd_vel_t)
        current_pose.orientation.w = w
        current_pose.orientation.x = x
        current_pose.orientation.y = y
        current_pose.orientation.z = z

        #publish odom message
        odom = Odometry()
        odom.pose.pose = current_pose

        self.dead_reckoning_odom_publisher.publish(odom)

        #publish path values
        path = Path()
        self.path_arr.append(current_pose)
        path.poses = self.path_arr
        self.dead_reckoning_path_publisher.publish(path)

def convertEulerToQuaternion(roll,pitch,yaw):
    # deal with quaternions, based on code from
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
    # This could be done more elegantly with a service call, but the instructions state only one python file
    
    cr = m.cos(roll * 0.5)
    sr = m.sin(roll * 0.5)
    cp = m.cos(pitch * 0.5)
    sp = m.sin(pitch * 0.5)
    cy = m.cos(yaw * 0.5)
    sy = m.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w,x,y,z

def main(args=None):
    rclpy.init(args=args)
    estimator_node = DeadReckoner()
    rclpy.spin(estimator_node)

    estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()