import sys
from math import cos, sin, pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
import csv

class Position:
    x = 0
    y = 0
    theta = 0

class EncVal:
    deltaR = 0
    deltaL = 0


def callback_scan(msg):
    global encVal
    global encVal1
    global start
    print("===============My joint start==================")
    msg_pos=list(msg.position)
    #print("Position message: ", msg_pos)
    msg_vel=list(msg.velocity)
    #print("Velocity message: ", msg_vel)
    T = msg.header.stamp.sec +(msg.header.stamp.nanosec/1000000000)
    #print("Time:             ", T)
    
    if start == 0:
        encVal1.deltaL = msg_pos[0]
        encVal1.deltaR = msg_pos[1]
        start = 1

    encVal.deltaL = msg_pos[0] - encVal1.deltaL
    encVal.deltaR = msg_pos[1] - encVal1.deltaR

    if abs(encVal.deltaL) > 2**15:
        if encVal.deltaL > 0:
            encVal.deltaL -= 2**16
        else:
            encVal.deltaL += 2**16
    
    if abs(encVal.deltaR) > 2**15:
        if encVal.deltaR > 0:
            encVal.deltaR -= 2**16
        else:
            encVal.deltaR += 2**16

    encVal1.deltaL = msg_pos[0]
    encVal1.deltaR = msg_pos[1]
    
    #print(encVal.deltaL, ' ', encVal.deltaR)
    
    Odometry_from_velocity(msg_vel[0], msg_vel[1])

    Odometry_from_position()
    
    print("===============My joint end==================")

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
    print(f"({pos.x},{pos.y},{yaw_from_quaternion(msg.pose.pose.orientation)})")
  




def Odometry_from_velocity(vl, vr):
    global pos
    L = 330
    a = [[cos(pos.theta)/2.0, cos(pos.theta)/2.0],[sin(pos.theta)/2.0, sin(pos.theta)/2.0],[1/L, -1/L]]
    v = [[vr], [vl]]

    V = np.dot(v, 0.1)
    A = np.dot(a, V)
    
    pos.x = pos.x + A[0]
    pos.y = pos.y + A[1]
    pos.theta = pos.theta + A[2]
    print("MyPosition (vel):       ", pos.x/1000, ' ', pos.y/1000, ' ', pos.theta)


def Odometry_from_position():
    global encVal
    global pos1
    d = 195
    L = 330
    ticksPerRev = 76600
    a = [[cos(pos1.theta)/2.0, cos(pos1.theta)/2.0],[sin(pos1.theta)/2.0, sin(pos1.theta)/2.0],[1/L, -1/L]]
    v = [[(encVal.deltaR/ticksPerRev)*pi*d], [(encVal.deltaL/ticksPerRev)*pi*d]]

    A = np.dot(a, v)

    pos1.x = pos1.x + A[0]
    pos1.y = pos1.y + A[1]
    pos1.theta = pos1.theta + A[2]

    print("MyPosition (pos):       ", pos1.x/1000, ' ', pos1.y/1000, ' ', pos1.theta) 





    

def main(args=None):
    global pos
    pos = Position()

    global pos1
    pos1 = Position()

    global encVal
    encVal = EncVal()

    global encVal1
    encVal1 = EncVal()

    global start
    start = 0

    rclpy.init()
    node = Node('listener')

    # Subscribe topics and bind with callback functions
    node.create_subscription(JointState, f"/pioneer5/joint_states", callback_scan, 10)
    node.create_subscription(Odometry, f"/pioneer5/odom", callback_odom, 10)
    # spin(node) simply keeps python from exiting until this node is stopped
    rclpy.spin(node)
    #rclpy.spin()

    node.destroy_node()
    rclpy.shutdown()


def read_from_csv():
    with open('odom_forward.csv') as csvfile:
        reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
        for row in reader:
            print(row)


if __name__ == '__main__':
    #main()
    read_from_csv()