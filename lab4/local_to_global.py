import json
import sys
import numpy as np
from math import sin, cos, acos
import scipy
import math
from scipy.optimize import fmin_cg
from matplotlib import pyplot as plt

pose = [[0.5586947070822248, 0.6511713988881517, 57.97214329552892],[0.6465763716613563, 0.7150069719014344, 89.72656896961927], \
       [0.7989620831986594, 0.43977675072769545, 99.25458],  [0.5154200765953387, 0.8358625075521338, 68.10270372691767], [1.1241869494099048, 1.2077680340604822, 95.59997281479365]]


json_data = open('line_localization_1.json')
data = json.load(json_data)


robot_pose = []
odom_0 = pose[0]
odom_world = []

for i in range(0,5):
    print('Calculated pose: ', pose[i])
    odom_world.append( [-data[i]['pose'][0] + odom_0[0], -data[i]['pose'][1] + odom_0[1], -data[i]['pose'][2] + odom_0[2] ] )
    print('Odom position in world frame: ', odom_world[i])
    print('Error: ', [ odom_world[i][0] - pose[i][0], odom_world[i][1] - pose[i][1], odom_world[i][2] - pose[i][2] ] )
    print(' ')

    plt.xlim(0, 1.25)
    plt.ylim(0, 1.25)
    plt.grid()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Dataset %d' % i)
    plt.plot( odom_world[i][0], odom_world[i][1], marker="x", markersize=9, markeredgecolor="red", label = 'Odometry')
    plt.plot( pose[i][0], pose[i][1], marker="o", markersize=9, markeredgecolor="blue", label = 'Feature')
    plt.legend(loc="upper left")
    plt.show()

