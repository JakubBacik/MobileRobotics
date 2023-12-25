import time 
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist     
from nav_msgs.msg import Odometry     
import numpy as np              
import threading
 
class RobotController:
  def __init__(self, Kp, Ki, Kd, dt, arrive_distance, angle_distance, desiredV):
      self.Kp = Kp
      self.Ki = Ki
      self.Kd = Kd
      self.dt = dt
      self.arrive_distance = arrive_distance
      self.angle_distance = angle_distance
      self.desiredV = desiredV
      self.E = 0
      self.old_e = 0

  def iteratePID(self):
      # Difference in x and y
      d_x = self.goal[0] - self.current[0]
      d_y = self.goal[1] - self.current[1]

      # Angle from robot to goal
      g_theta = np.arctan2(d_y, d_x)
      #g_theta = self.goal[2]

      # Error between the goal angle and robot angle
      alpha = g_theta - self.current[2]
      e = np.arctan2(np.sin(alpha), np.cos(alpha))

      e_P = e
      e_I = self.E + e
      e_D = e - self.old_e

      # Angular velocity
      w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
      w = np.arctan2(np.sin(w), np.cos(w))

      self.E = self.E + e
      self.old_e = e

     # Linear velocity
      distance = np.sqrt(d_x**2 + d_y**2)
      if distance > 1:
          v = 5*self.desiredV
      elif distance > 0.5:
          v = self.desiredV
      elif distance > 0.1:
          v = 0.5*self.desiredV
      elif distance > 0.05:
          v = 0.2*self.desiredV
      else:
          v = 0.0 
      #v = self.desiredV if distance > self.arrive_distance else 0.0


      return v, w


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
