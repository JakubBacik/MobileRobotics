import time 
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist     
from nav_msgs.msg import Odometry     
import numpy as np              
import threading
from matplotlib import pyplot as plt
import time
from publisher_subscriber_ros import laser_node

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
      if distance > 0.8:
          v = 3*self.desiredV
      elif distance > 0.3:
          v = self.desiredV
      elif distance > 0.1:
          v = 0.5*self.desiredV
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

def isArrived(current, goal, arrive_distance=0.1):
  current_state = [current[0], current[1]]
  goal_state = [goal[0], goal[1]]
  distance_err = math.dist(current_state, goal_state)
  if distance_err < arrive_distance:
      return True
  else:
      return False
       
 
def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle        
                      
def calculate_angle(path):
    theta123 = 0
    theta1 = 0
    angle_tab = []

    for i in range(len(path)-1):
      angle = np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])
      angle_tab.append(angle)

    # for i in range(1, len(angle_tab)-1):
    #   if angle_tab[i] >= 3.1 and  angle_tab[i-1] > 0:
    #     angle_tab[i] = np.pi
    #   elif angle_tab[i] >= 3.1 and angle_tab[i-1] < 0:
    #     angle_tab[i] = -np.pi
       
        # theta123 = np.degrees(np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0]))
        # diff = normalize_angle(theta123 - theta1)
        # if diff != 0:
        #     theta1 = theta123
        #     if diff > 0:
        #         angle_tab.append(np.pi/2)
        #     elif diff < -90:
        #         angle_tab.append(-np.pi)
        #     else:
        #         angle_tab.append(-np.pi/2)
        # else:
        #     angle_tab.append(0.0)

    return angle_tab

class AngularController:
    def __init__(self, Kp, Ki, Kd, dt, desired_w):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.desired_w = desired_w  # Desired angular velocity

        # Initialize the integral and previous error terms
        self.E = 0.0
        self.old_e = 0.0

    def control(self, error_angle):
        # Proportional term
        e_P = error_angle

        # Integral term
        self.E += error_angle * self.dt
        e_I = self.Ki * self.E

        # Derivative term
        e_D = self.Kd * (error_angle - self.old_e) / self.dt
        self.old_e = error_angle

        # Angular velocity
        w = self.Kp*e_P + e_I + e_D
        w = np.arctan2(np.sin(w), np.cos(w))  # Wrap to [-pi, pi]

        # Limit the angular velocity to the desired value
        if w > self.desired_w:
            w = self.desired_w
        elif w < -self.desired_w:
            w = -self.desired_w

        return w
    
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.isum = 0.0
        self.limit = limit
        self.e_prev = 0.0
        

    def calc(self, desired, actual):
        e = desired - actual
        de = e - self.e_prev

        self.isum = self.isum + e

        output = self.kp*e + self.ki*self.isum + self.kd*de
        

        if(output > self.limit):
            output = self.limit
        if(output < -self.limit):
            output = -self.limit
          
        self.e_prev = e


        return output    

def pol2Car(r, theta, pose):
    x = r * np.cos(theta+pose[2]) + pose[0]
    y = r * np.sin(theta+pose[2]) + pose[1]
    point = np.matrix([[x],[y]])

    return point

def main(args=None):
  angle = 0
  rclpy.init(args=args)
  velocity_publisher = VelocityPublisher()
  laser_raw_data = laser_node()

  Odometry = Odometry_DataSubscriber()
  executor = rclpy.executors.MultiThreadedExecutor()
  executor.add_node(Odometry)
  executor.add_node(laser_raw_data)
  executor_thread = threading.Thread(target=executor.spin, daemon=True)
  executor_thread.start()

  rate = Odometry.create_rate(10)

  i = 0
  #goals= [[0.0, 0.0, 0.0], [0.0, 0.0, -np.pi], [0.0, 0.0, np.pi], [0.0, 0.0, -np.pi], [0.0, 0.0, 0.0], [0.0, 0.0, np.pi/2], [0.0, 0.0, 0.0], [0.0, 0.0, -np.pi/2], [0.0, 0.0, 0.0]]
  goals = [[0.0, 0.0, 0.0],[1.0, 0.0, np.pi/2],[1.0,1.0,0.0],[2.0,1.0,0],[3.0,1.0, -np.pi/2],[3.0, 0.0, -np.pi],[2.0, 0.0, -np.pi/2],[2.0, -1.0, -np.pi],[1.0, -1.0, -np.pi],[0.0, -1.0, np.pi/2],[0.0, 0.0, np.pi/2],[0.0, 1.0, np.pi],[-1.0, 1.0, -np.pi/2],[-1.0,0.0, -np.pi],[-2.0, 0.0, -np.pi]]
  # goals = [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.5, 0.5, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.0]]
  # goals = [[0.0, 0.0, 0.0],[0.0, 0.5, 0.0],[-0.5, 0.5, 0.0],[-0.5, 0.0], [0.0, 0.0, 0.0]]
  #goals = [[0.0, 0.0, 0.0], [0.0, -0.5, 0.0],[0.5, -0.5, 0.0],[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]]
  #goals = [[0.0, 0.0, 0.0], [-0.5, 0.0, 0.0],[-1.0, 0.0, 0.0],[-1.5, 0.0, 0.0],[-2.0, 0.0, 0.0]]
  #goals = [[0.0, 0.0, 0.0], [0.0, 0.0, np.pi],[0.0, 0.0, -np.pi/2],[0.0, 0.0, -np.pi], [0.0, 0.0, np.pi/2]]
  # angle = calculate_angle(goals)
  # angle.append(angle[-1])
  arrive_distance = 0.09
  try:
    while rclpy.ok():
       rate.sleep()
       rate.sleep()
       rate.sleep()
       min_angle = -2.356100082397461
       max_angle = 2.0932999382019043
       angle_increment = 0.006144199054688215


       theta = np.arange(min_angle, max_angle, angle_increment)
       dataScan = laser_raw_data.lidar_buf

       if dataScan != 0:
        dataScan = dataScan.to_list()
        for k in range(len(dataScan)):
                if np.isinf(dataScan[k]) or np.isnan(dataScan[k]):
                    continue  

                pt = pol2Car(dataScan[k], theta[k], Odometry.pose)
                plt.plot(pt[1], pt[0], 'bo')
        plt.show()
        
    #   if(i< len(goals)):
    #         goal = goals[i][0:2]
    #         current = [Odometry.pose[0], Odometry.pose[1], Odometry.pose[2]] 
    #         distance_to_goal = np.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
    #         linearController = PID(kp = 0.09, ki =0.015, kd=0.001, limit=0.5)
    #         angularController = PID(kp = 0.2, ki =0.005, kd=0.0001, limit=0.1)
    #         d_x = goal[0] - current[0]
    #         d_y = goal[1] - current[1]

    #         # Angle from robot to goal
    #         desired_angle = np.arctan2(d_y, d_x)

    #     # ...

    #     # Calculate the current angle error
              
    #         current_angle = Odometry.pose[2] 

            
    #         # Handle the discontinuity at -π and π
    #         if abs(desired_angle - current_angle) > np.pi:
    #             if desired_angle > current_angle:
    #                 desired_angle -= 2 * np.pi
    #             else:
    #                 current_angle -= 2 * np.pi

    #         # Calculate the difference between the desired and current angles
    #         error_angle = desired_angle - current_angle

    #         w = angularController.calc(error_angle, 0)
    #         v = linearController.calc(distance_to_goal, 0)

    #         print("seting  goal",i," | ", goal, " >>", current)
    #         velocity_publisher.publish_velocity(v, w) 
    #         rate.sleep()
            

    #         if distance_to_goal < arrive_distance:
    #           velocity_publisher.publish_velocity(0.0, 0.0)
    #           angularController = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)
    #           desired_angle = goals[i][2]  # Set your desired angle here
    #           angle_tolerance = 0.01
    #           while True:
    #             # Calculate the current angle error
    #             current_angle = Odometry.pose[2]  # Assuming this is the current angle
    #             error_angle = desired_angle - current_angle
    #             print("seting  angle",i," | ", desired_angle, " >>>>", current_angle)


    #             # Wrap the angle error to the range [-pi, pi]
    #             # error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

    #              # Handle the discontinuity at -π and π
    #             if abs(desired_angle - current_angle) > np.pi:
    #                 if desired_angle > current_angle:
    #                     desired_angle -= 2 * np.pi
    #                 else:
    #                     current_angle -= 2 * np.pi

    #             # Calculate the error
    #             error_angle = desired_angle - current_angle
    #             # If the error is within the tolerance, break the loop
    #             if abs(error_angle) < angle_tolerance:
    #                 velocity_publisher.publish_velocity(0.0, 0.0)
    #                 rate.sleep() 
    #                 rate.sleep()
    #                 rate.sleep()
    #                 rate.sleep()
    #                 i = i+1
    #                 break

    #             # Calculate angular velocity using the angular controller
    #             # w = angularController.calc(desired_angle, current_angle)
    #             w = angularController.calc(error_angle, 0)

    #             velocity_publisher.publish_velocity(0.0, w) 
    #             rate.sleep() 

              
      
    #   if i == len(goals):
    #     break

    # plt.show()
  except KeyboardInterrupt:
    pass


  executor_thread.join()

  
  velocity_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
     main()
























#===================================================================================================== One point test 
# import matplotlib.pyplot as plt

# def main():
#     # Create a RobotController
#     controller = RobotController(Kp=0.5, Ki=0.05, Kd=0.02, dt=0.1, arrive_distance=0.1, angle_distance=0.1, desiredV=0.5)

#     # Set the current and goal positions
#     controller.current = [0.0, 0.0, -1.57]  # x, y, angle
#     controller.goal = [0.5, 0.5, 1.0]  # x, y, angle

#     # Prepare the plot
#     plt.figure()
#     plt.plot(controller.current[0], controller.current[1], 'go')  # Start position
#     plt.plot(controller.goal[0], controller.goal[1], 'ro')  # Goal position

#     # Control the robot until. it reaches the goal
# # Control the robot until it reaches the goal
#     for i in range(500):
#         v, w = controller.iteratePID()

#         # Calculate the angle error
#         error_x = controller.goal[0] - controller.current[0]
#         error_y = controller.goal[1] - controller.current[1]
#         error_angle = np.arctan2(error_y, error_x) - controller.current[2]

#         # Wrap the angle error to the range [-pi, pi]
#         error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

#         # If the absolute value of the angle error is greater than pi/2, make a U-turn
#         if abs(error_angle) > np.pi / 3.1:
#             print("Making a U-turn")
#             v = 0
#             w = np.sign(w) #* controller.desiredV  # Use the maximum angular velocity for the U-turn

#         # Update the robot's position and angle
#         controller.current[0] += v * controller.dt * np.cos(controller.current[2])
#         controller.current[1] += v * controller.dt * np.sin(controller.current[2])
#         controller.current[2] += w * controller.dt

#         # Print the velocities
#         print(f"{i}")

#         # Update the plot
#         plt.plot(controller.current[0], controller.current[1], 'b.')  # Current position

#         distance_to_goal = np.sqrt((controller.goal[0] - controller.current[0])**2 + (controller.goal[1] - controller.current[1])**2)
#         if distance_to_goal < controller.arrive_distance:
#             break

#     # Show the plot
#     plt.show()

# if __name__ == "__main__":
#     main()

# def main():
#     controller = RobotController(Kp=7.0, Ki=0.5, Kd=0.01, dt=0.1, arrive_distance=0.1, angle_distance=0.1, desiredV=0.1)
#     controller.current = [5.0, 0.5, 1.0]  # x, y, angle
#     goals = [[0.0, 0.0, 0], [0.5, 0.0, 0], [0.5, 0.0, 0], [1.5, 0.0, 0], [1.0, 0.0, 0], [2.5, 0.0, 0], [1.5, 0.0, 0], [3.5, 0.0, 0], [4.0, 0.0, 0], [4.5, 0.0, 0], [5.0, 0.0, 0], [5.0, 0.5, 0], [5.0, 0.5, 0], [5.0, 1.5, 0], [5.0, 1.0, 0], [5.0, 2.5, 0], [5.0, 1.5, 0], [5.0, 3.5, 0], [5.0, 4.0, 0], [5.0, 4.5, 0], [5.0, 5.0, 0], [5.0, 5.5, 0], [5.0, 6.0, 0], [5.0, 6.5, 0], [5.0, 7.0, 0], [5.0, 7.5, 0], [5.0, 8.0, 0], [5.0, 8.5, 0], [5.0, 9.0, 0], [5.0, 9.5, 0]]
#     plt.figure()
#     plt.plot(controller.current[0], controller.current[1], 'go')  # Start position

#     for i in range(len(goals)):
#         controller.goal = goals[i]
#         plt.plot(controller.goal[0], controller.goal[1], 'ro')  # Goal position

#         while True:
#             v, w = controller.iteratePID()

#             # Calculate the angle error
#             error_x = controller.goal[0] - controller.current[0]
#             error_y = controller.goal[1] - controller.current[1]
#             error_angle = np.arctan2(error_y, error_x) - controller.current[2]

#             # Wrap the angle error to the range [-pi, pi]
#             error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

#             # If the absolute value of the angle error is greater than pi/2, make a U-turn
#             if abs(error_angle) > np.pi / 2:
#                 print("Making a U-turn")
#                 v = 0.0
#                 w = np.sign(w)  # Use the maximum angular velocity for the U-turn
#                 time.sleep(1)

#             # Update the robot's position and angle
#             controller.current[0] += v * controller.dt * np.cos(controller.current[2])
#             controller.current[1] += v * controller.dt * np.sin(controller.current[2])
#             controller.current[2] += w * controller.dt

#             plt.plot(controller.current[0], controller.current[1], 'b.')  # Current position

#             distance_to_goal = np.sqrt((controller.goal[0] - controller.current[0])**2 + (controller.goal[1] - controller.current[1])**2)
#             if distance_to_goal < controller.arrive_distance:
#                 break

#     plt.show()

# if __name__ == "__main__":
#     main()