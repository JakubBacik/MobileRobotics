from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped     
from nav_msgs.msg import Odometry, OccupancyGrid, Path  
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


class map_msg(Node):
    def __init__(self):
        super().__init__('MapMsg')
        self.publisher_robot_state = self.create_publisher(OccupancyGrid, '/map_msg', 1)

    def publish_map(self, map_txt):
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'odom'
        map_msg.info.resolution = 0.1
        map_msg.info.width = 250
        map_msg.info.height = 250
        map_msg.info.origin.position.x = -12.5
        map_msg.info.origin.position.y = -12.5
        map_msg.data = map_txt

        self.publisher_robot_state.publish(map_msg)

class obstacle_map_msg(Node):
    def __init__(self):
        super().__init__('ObstacleMapMsg')
        self.publisher_robot_state1 = self.create_publisher(OccupancyGrid, '/map_msg_obstacle', 1)

    def publish_map(self, map_txt):
        map_msg1 = OccupancyGrid()
        map_msg1.header.frame_id = 'odom'
        map_msg1.info.resolution = 0.5
        map_msg1.info.width = 50
        map_msg1.info.height = 50
        map_msg1.info.origin.position.x = -12.5
        map_msg1.info.origin.position.y = -12.5
        map_msg1.data = map_txt

        self.publisher_robot_state1.publish(map_msg1)

class path_map_msg(Node):
    def __init__(self):
        super().__init__('PathMapMsg')
        self.publisher_robot_state1 = self.create_publisher(Path, '/path_map_msg', 1)
    
    def publish_path(self,path):
        msg = Path()
        msg.header.frame_id = "odom"

        for i in path:
            pose1 = PoseStamped()
            pose1.pose.position.x = float(i[0])
            pose1.pose.position.y = float(i[1])

            msg.poses.append(pose1)

        self.publisher_robot_state1.publish(msg)    


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


