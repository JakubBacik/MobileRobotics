import time 
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist     
from nav_msgs.msg import Odometry     
import numpy as np              
import threading
 
class Controller():
  def __init__(
          self, start_, goal_, 
          kP=0.5, kI=0.02, kD=0.01, dT=0.1, v=0.1,
          arrive_distance=0.1):

      self.current = start_
      self.goal = goal_

      self.E = 0   # Cummulative error
      self.old_e = 0  # Previous error

      self.Kp = kP
      self.Ki = kI
      self.Kd = kD

      self.desiredV = v
      self.dt = dT  # in second
      self.arrive_distance = arrive_distance


  def iteratePID(self):
      # Difference in x and y
      d_x = self.goal[0] - self.current[0]
      d_y = self.goal[1] - self.current[1]

      # Angle from robot to goal
      g_theta = np.arctan2(d_y, d_x)

      # Error between the goal angle and robot angle
      alpha = g_theta - self.current[2]
      # alpha = g_theta - math.radians(90)
      e = np.arctan2(np.sin(alpha), np.cos(alpha))

      e_P = e
      e_I = self.E + e
      e_D = e - self.old_e

      # This PID controller only calculates the angular
      # velocity with constant speed of v
      # The value of v can be specified by giving in parameter or
      # using the pre-defined value defined above.
      w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

      w = np.arctan2(np.sin(w), np.cos(w))

      self.E = self.E + e
      self.old_e = e
      v = self.desiredV

      return v, w


  def isArrived(self):
      current_state = [self.current[0], self.current[1]]
      goal_state = [self.goal[0], self.goal[1]]

      distance_err = math.dist(current_state, goal_state)
      if distance_err < self.arrive_distance:
          return True
      else:
          return False


class Odometry_DataSubscriber(Node):
  def __init__(self):
    super().__init__('Odom_data_subscriber')
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
      
class VelocityPublisher(Node):
  def __init__(self):
    super().__init__('Velocity_Publisher')
    self.publisher_ = self.create_publisher(Twist, '/demo/cmd_vel', 10)

  def publish_velocity(self,v,w):
    velocity_command = Twist()
    velocity_command.linear.x = v  # Replace this with your desired linear velocity
    velocity_command.angular.z = w  # Replace this with your desired angular velocity

    self.publisher_.publish(velocity_command)
    # self.get_logger().info('Published velocity command: linear={}, angular={}'.format(
    #     velocity_command.linear.x, velocity_command.angular.z))       
        
