import json
from scipy import optimize
from matplotlib import collections  as mc
import pylab as pl
import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos



multistep = 5
treshold = 0.03#0.02
max_dist = 2
cancel_len = 0.5



def euclidDist(p1,p2):
	return np.sqrt((p1[0] -p2[0])**2 + (p1[1] -p2[1])**2)



def shortest_distance(x1, y1, a, b, c):    
    d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
    return d



def distance_beetwen_to_point(line):
    return math.sqrt( (line[1][0] - line[0][0])**2 + (line[1][1] - line[0][1])**2 )



def func(x, a, b):
    y = a*x + b
    return y



def line_fitting(x, y, start):
    if start+multistep >= len(x):   #prevent out of bound index
        return len(x)-1

    for j in range(start, start+multistep):
        if scan_data[j] > max_dist:
            return j

    for i in range(start+multistep, len(x)):
        if scan_data[i] > max_dist:
            break

        alpha = optimize.curve_fit(func, xdata = x[start:i], ydata = y[start:i])[0]
        
        if shortest_distance(x[i], y[i], alpha[0], -1, alpha[1]) > treshold: 
            break

    return i



def line_recognition( scan_data, robot_position):
    idx = 0
    lines = []
    index_of_lidar = []
    x, y = get_xy( scan_data, robot_position)

    while idx < len(x)-1:
        line_tmp = []
        indx_tmp = []

        while scan_data[idx] > max_dist:
            if idx < len(x)-1:
                idx += 1
            else:
                break
        
        line_tmp.append( (x[idx],y[idx]) )
        indx_tmp.append(idx)
        idx = line_fitting(x,y,idx)
        indx_tmp.append(idx-1)
        line_tmp.append( (x[idx-1], y[idx-1]) )

        lines.append(line_tmp)
        index_of_lidar.append(indx_tmp)

    return lines, index_of_lidar



def choose_corect_line(lines):
    correct_line = []

    for i in range(len(lines)):
        if distance_beetwen_to_point(lines[i]) > cancel_len and distance_beetwen_to_point(lines[i]) < 100:
            correct_line.append(lines[i])
            
    return correct_line



def get_xy(scan_data, robot_position):
    x = [] 
    y = []   

    theta = (pl.pi/512 )*(pl.arange(0,512)-256)  # angle in radians

    for i in range(0, 512):
        x.append(scan_data[i] * math.cos(-theta[i]+robot_position[2]) + robot_position[0])
        y.append(scan_data[i] * math.sin(-theta[i]+robot_position[2]) + robot_position[1])

    return x, y



def pol2Car(r, theta):
	x = r * cos(theta)
	y = r * sin(theta)
	point = np.matrix([[x],[y]])
	return point



def f(z, *args):
	x,y=z
	x1, y1, x2, y2, d1, d2 = args
	return ((((x1-x)**2)+((y1-y)**2)-d1**2)**2)+((((x2-x)**2)+((y2-y)**2)-d2**2)**2)



def get_raw_data():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="name of the json data file")
    parser.add_argument("number", help="database set number")
    args = parser.parse_args()

    json_data = open(args.filename)
    data = json.load(json_data)
    dataset = int(args.number)

    return data[dataset]["scan"], data[dataset]["pose"]



####################################################################### Start 

scan_data, robot_position = get_raw_data()

lines, index_of_lidar = line_recognition( scan_data, robot_position)
correct_line = choose_corect_line(lines)

lc = mc.LineCollection(correct_line, linewidths=2)
fig, ax = pl.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

x, y = get_xy(scan_data, robot_position)

pl.plot(x, y, 'r.', markersize = 1)#, color='green')
pl.xlabel('x')
pl.ylabel('y')
pl.axis('equal')
pl.show()


## Distance point from line
p1 = np.polyfit(correct_line[0][0], correct_line[0][1],1)
p2 = np.polyfit(correct_line[1][0], correct_line[1][1],1)


robot_pose=[0,0]
robot_pose[1] = abs(p1[1])/np.sqrt(p1[0]**2 + 1)
robot_pose[0] = abs(p2[1])/np.sqrt(p2[0]**2 + 1)


lengthOfLine1 = euclidDist(correct_line[0][0], correct_line[0][1])
lengthOfLine2 = euclidDist(correct_line[1][0], correct_line[1][1])
			
axes = [[[0 ,0],[0, lengthOfLine1 ]],[[0,0],[lengthOfLine2, 0]]]

lc = mc.LineCollection(axes, linewidths=2)
ax = pl.subplots()
ax = plt.subplot(1, 1, 1)
ax.add_collection(lc)
ax.axis('equal')
ax.margins(0.1)
ax.scatter(robot_pose[0], robot_pose[1])
			
    
print(index_of_lidar)

alpha1 = (np.pi/512 )*index_of_lidar[1][1]
alpha2 = (np.pi/512 )*index_of_lidar[1][0]

d1 = scan_data[index_of_lidar[1][1]]
d2 = scan_data[index_of_lidar[1][0]] 

p1 = pol2Car(d1, alpha1)
p2 = pol2Car(d2, alpha2)

d0 = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
areaOfTriangle = d1 * d2 * np.sin(np.abs(alpha1 - alpha2)) / 2

hate = areaOfTriangle * 2 / d0
y = hate;

beta = np.arcsin(areaOfTriangle * 2 / (d1 * d0));
x = hate / np.tan(beta);

d2ForRightAngled = np.sqrt(d1**2 + d0**2)
if (d2 > d2ForRightAngled):
	x = -x

print("geometric_aproach :", x.item(0,0), " ", y.item(0,0))


gamma = np.arcsin(x / d2)
theta = alpha1 + gamma - np.pi/2;
print("geometric_aproach theta: ", (theta[0][0]*180)/math.pi)
first_point = float(x.item(0,0))
second_point = float(y.item(0,0))
ax.scatter(first_point, second_point)
plt.show()
