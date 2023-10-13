import sys
import csv
import rclpy
import numpy as np
from math import cos, sin, pi
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class Position:
    x = 0
    y = 0
    theta = 0

class Encoder_value:
    delta_r = 0
    delta_l = 0


def callback_scan(msg):
    global position_odom_vel, position_odom_pos
    global enc_val_previous, enc_val_current
    global start_for_encoder, time_previous, time_current
    
    msg_pos = list(msg.position)
    msg_vel = list(msg.velocity)
    msg_time = msg.header.stamp.sec +(msg.header.stamp.nanosec/1000000000)

    if start_for_encoder == 0:
        enc_val_previous.delta_l = msg_pos[0]
        enc_val_previous.delta_r = msg_pos[1]
        time_previous = msg_time 
        start_for_encoder = 1

    enc_val_current.delta_l = msg_pos[0] - enc_val_previous.delta_l
    enc_val_current.delta_r = msg_pos[1] - enc_val_previous.delta_r
    time_current = msg_time - time_previous

    if abs(enc_val_current.delta_l ) > 2**15:
        if enc_val_current.delta_l  > 0:
            enc_val_current.delta_l -= 2**16
        else:
            enc_val_current.delta_l  += 2**16
    
    if abs(enc_val_current.delta_r) > 2**15:
        if enc_val_current.delta_r > 0:
            enc_val_current.delta_r -= 2**16
        else:
            enc_val_current.delta_r += 2**16

    enc_val_previous.delta_l = msg_pos[0]
    enc_val_previous.delta_r = msg_pos[1]
    time_previous = msg_time 

    odometry_fv = odometry_from_velocity(position_odom_vel, msg_vel[0], msg_vel[1], time_current)
    odometry_fp = odometry_from_position(position_odom_pos, enc_val_current.delta_l, enc_val_current.delta_r)
    print("Odometry velocity:", odometry_fv.x/1000, ' ', odometry_fv.y/1000, ' ', odometry_fv.theta)
    print("Odometry position:", odometry_fp.x/1000, ' ', odometry_fp.y/1000, ' ', odometry_fp.theta)
    


def yaw_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return yaw

def callback_odom(msg):
    pos=msg.pose.pose.position
    print(f"Odometry ({pos.x},{pos.y},{yaw_from_quaternion(msg.pose.pose.orientation)})")

    print("============================================================================")

def read_from_csv():
    global position_odom_vel, position_odom_pos
    global enc_val_previous, enc_val_current
    global start_for_encoder, time_previous, time_current

    with open('CSV_file/odom_forward.csv') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
        for row in reader:           
            if start_for_encoder == 0:
                enc_val_previous.delta_l = float(row['pos_l'])
                enc_val_previous.delta_r = float(row['pos_r[ticks]'])
                time_previous = float(row['#time[s]'])
                start_for_encoder = 1

            enc_val_current.delta_l = float(row['pos_l']) - enc_val_previous.delta_l
            enc_val_current.delta_r = float(row['pos_r[ticks]']) - enc_val_previous.delta_r
            time_current = float(row['#time[s]'])- time_previous

            if abs(enc_val_current.delta_l ) > 2**15:
                if enc_val_current.delta_l  > 0:
                    enc_val_current.delta_l -= 2**16
                else:
                    enc_val_current.delta_l  += 2**16
            
            if abs(enc_val_current.delta_r) > 2**15:
                if enc_val_current.delta_r > 0:
                    enc_val_current.delta_r -= 2**16
                else:
                    enc_val_current.delta_r += 2**16

            enc_val_previous.delta_l = float(row['pos_l'])
            enc_val_previous.delta_r = float(row['pos_r[ticks]'])
            time_previous = float(row['#time[s]'])

            odometry_fv = odometry_from_velocity(position_odom_vel, float(row['vel_l']), float(row['vel_r[mm/s]']), time_current)
            odometry_fp = odometry_from_position(position_odom_pos, enc_val_current.delta_l, enc_val_current.delta_r)
            print("Odometry velocity:", odometry_fv.x/1000, ' ', odometry_fv.y/1000, ' ', odometry_fv.theta)
            print("Odometry position:", odometry_fp.x/1000, ' ', odometry_fp.y/1000, ' ', odometry_fp.theta)
            print("======================== NEXT ITERATION ========================")

"""
Calculates odometry using wheel velocities for a differential drive robot.

Args:
    current_position (class): A tuple (x, y, theta) representing the current position and orientation of the robot. 
    velocity_l (float): The left wheel linear velocity.
    velocity_r (float): The right wheel linear velocity.
    delta_t (float): The time interval over which the velocities are measured.

Returns:
    class: position representing the estimated new position and orientation
    of the robot after applying the wheel velocities over the time interval 'delta_t'.
"""
def odometry_from_velocity(current_position, velocity_r, velocity_l, delta_t):
    dis_between_wheels = 330
    
    a_matrix = np.array([[cos(current_position.theta)/2.0, cos(current_position.theta)/2.0],
                         [sin(current_position.theta)/2.0, sin(current_position.theta)/2.0],
                         [1/dis_between_wheels, -1/dis_between_wheels]])
    v_matrix = np.array([[velocity_r], [velocity_l]])

    V = np.dot(v_matrix, delta_t)
    A = np.dot(a_matrix, V)
    
    current_position.x += A[0]
    current_position.y += A[1]
    current_position.theta += A[2]

    return current_position

"""
Calculate odometry based on changes in position and wheel movements.

Args:
    current_position (Position): A Position object representing the current position and orientation
        of the robot.
    delta_l (float): The change in distance for the left wheel .
    delta_r (float): The change in distance for the right wheel.

Returns:
    Position: A new Position object representing the estimated new position and orientation
    of the robot after applying the changes in wheel movements.

Note: This is a basic example and may not account for all real-world factors and kinematic model details.
"""
def odometry_from_position(current_position, delta_l, delta_r):
    wheel_diameter = 195
    dis_between_wheels = 330
    ticks_per_revolution = 76600

    a_matrix = np.array([[cos(current_position.theta)/2.0, cos(current_position.theta)/2.0],
                         [sin(current_position.theta)/2.0, sin(current_position.theta)/2.0],
                         [1/dis_between_wheels, -1/dis_between_wheels]])
    v_matrix = np.array([[(delta_r/ticks_per_revolution)*pi*wheel_diameter], 
                         [(delta_l/ticks_per_revolution)*pi*wheel_diameter]])

    A = np.dot(a_matrix, v_matrix)

    current_position.x += A[0]
    current_position.y += A[1]
    current_position.theta += A[2]

    return current_position

def main(args=None):
    global position_odom_vel, position_odom_pos
    position_odom_vel = Position()
    position_odom_pos = Position()

    global enc_val_previous, enc_val_current
    enc_val_previous = Encoder_value()
    enc_val_current  = Encoder_value()

    global start_for_encoder, time_previous, time_current
    start_for_encoder = 0
    time_previous = 0
    time_current = 0

    read_from_csv()
    
    # rclpy.init()
    # node = Node('listener')

    # node.create_subscription(JointState, f"/pioneer5/joint_states", callback_scan, 10)
    # node.create_subscription(Odometry, f"/pioneer5/odom", callback_odom, 10)

    # rclpy.spin(node)

    # node.destroy_node()
    # rclpy.shutdown()




if __name__ == '__main__':
    main()
    