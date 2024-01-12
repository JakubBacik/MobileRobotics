import numpy as np

'''
    This is a PID controller class. 
'''
class PID_linear:
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
          v = 2*self.desiredV
      elif distance > 0.3:
          v = self.desiredV
      elif distance > 0.1:
          v = 1*self.desiredV
      else:
          w=0.0
          v = 0.0 
      #v = self.desiredV if distance > self.arrive_distance else 0.0
      return v, w

'''
    This is a PID controller class for angular velocity. 
'''
class PID_angular:
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