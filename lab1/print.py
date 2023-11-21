import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from time import sleep

loop_count=25  # drawing loop iterations
beam_half_angle=2 # half of angular beam width

# 
# A function to calculate Cartesian coordinates to polar
#  result: a tuple (rho,phi)
#  rho - radius, phi - angle in degrees
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

# A function to transform polar  coordinates to Cartesian
# input angle in degrees
# returns a tuple (x,y)
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def plotarrows(ax,arrlist):
    y=[[0,0]+x for x in arrlist ]
    soa =np.array(y) 
    X,Y,U,V = zip(*soa)
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)


def callback_scan(msg):
    skan = list(msg.ranges)
    ax.cla()
    x = np.arange(0, 726)
    k = np.arange(-126, 600)
    theta = (1.4167*np.pi/726)*k
    data1 = []
    for i in x:
        data1.append(list(pol2cart(skan[i], theta[i])))

    plotarrows(ax,data1)
    ax.set_xlim([-3,3])
    ax.set_ylim([-3,3])
    plt.draw()
    plt.pause(0.0001)
    sleep(0.2)

plt.figure()
ax = plt.gca()
ax.set_aspect('equal')
ax.set_xlim([-2,2])
ax.set_ylim([-2,2])
plt.ion()
plt.show()

rclpy.init()
node = Node('listener')
node.create_subscription(LaserScan, f"/pioneer5/scan", callback_scan, 10)
rclpy.spin(node)
rclpy.shutdown()


