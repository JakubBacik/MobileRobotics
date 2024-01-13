from rclpy.node import Node
from geometry_msgs.msg import Twist     
from nav_msgs.msg import Odometry     
from sensor_msgs.msg import LaserScan 
import numpy as np

'''
    This file contains the publisher and subscriber nodes for the robot.
'''
class odometry_node(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.pose = [0, 0, 0]
        self.sub_odom = self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.get_logger().info('Odom Subscriber has been started.')

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = self.euler_from_quaternion(msg.pose.pose.orientation)[2]


'''
  This class is used to publish velocity commands to the robot.
'''
class velocity_node(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.pub_vel = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.get_logger().info('Velocity Subscriber has been started.')

    def publish_velocity(self, linear_velocity, angular_velocity):
        velocity_command = Twist()
        velocity_command.linear.x = linear_velocity
        velocity_command.angular.z = angular_velocity

        self.pub_vel.publish(velocity_command)


'''
  This class is used to subscribe to the laser scan data and odometry data.
'''
class laser_node(Node):
    def __init__(self):
        super().__init__('data_subscriber') 
        self.lidar_buf = 0

        self.sub_lidar = self.create_subscription(LaserScan, '/demo/laser/out', self.lidar_callback, 10)
        self.get_logger().info('Lidar Subscriber has been started.')


    def lidar_callback(self, msg):
        self.lidar_buf = msg.ranges


