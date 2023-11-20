import json
import sys
import numpy as np
from math import sin, cos, acos
import scipy
import math
from scipy.optimize import fmin_cg
from matplotlib import pyplot as plt

pose = [[0.5586947070822248, 0.6511713988881517, 57.97214329552892],[0.6465763716613563, 0.7150069719014344, 89.72656896961927], \
        [0.5154200765953387, 0.8358625075521338, 68.10270372691767],[0.7989620831986594, 0.43977675072769545, 0], [1.1241869494099048, 1.2077680340604822, 95.59997281479365]]


json_data = open('line_localization_1.json')
data = json.load(json_data)


robot_pose = []

for i in range(0,5):
    print(data[i]['pose'])

x=pose[0][0]
y=pose[0][1]
theta = 135

for i in range(0,5):
    robot_position = data[i]['pose']
    x = (robot_position[0] * math.cos(math.radians(theta + robot_position[2]))) + x
    y = (robot_position[1] * math.sin(math.radians(theta + robot_position[2]))) + y
    robot_pose.append([x, y])


plt.xlim(-1.25, 1.25)
plt.ylim(-1.25, 1.25)
plt.grid()
for i in range(0,5):
    plt.plot(robot_pose[i][1], robot_pose[i][0], marker="x", markersize=9, markeredgecolor="red")
plt.show()