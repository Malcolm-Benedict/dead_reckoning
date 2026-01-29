import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class Deadreckoner(Node):
    def __init__(self):
        super().__init__('dead_reckoner')
        self.dead_reckoning_path_publisher = self.create_publisher(Path, '/dead_reckoning/path', 10)
        #qos_profile = QoSProfile(
        #    depth=10,  
        #    reliability=ReliabilityPolicy.BEST_EFFORT
        #)
        self.dead_reckoning_odom_publisher = self.create_publisher(Odometry, '/dead_reckoning/odom', 10)

        # create subscriber with "best effort" setting to match turtlebot 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.range_callback, 
            qos_profile)
        
def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarProcessor()
    rclpy.spin(lidar_node)

    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()