import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
import json
from numpy.linalg import norm
import random
import math
import pylab as pl
from matplotlib import collections  as mc

def open_json_robot_data(name, data_set_number):	
	json_data = open(name)
	data_load = json.load(json_data)

	data=data_load[data_set_number]["scan"]

	for i in range(0, len(data)):
		if data[i]> 2:
			data[i] = 100

	return data

def open_json_robot_position(name, data_set_number):
	json_data = open(name)
	data_load = json.load(json_data)

	robot_position = data_load[data_set_number]["pose"]

	return robot_position

def open_raw_data_for_plot(name, data_set_number):
	json_data = open(name)
	data = json.load(json_data)

	data = data[data_set_number]["scan"]

	x = np.arange(0,512)
	theta = (np.pi/512 )*x 

	data_to_return = np.zeros((2,512))
	robot_position = open_json_robot_position(name, data_set_number)

	for x in np.arange(0,512):
		point = pol_to_car(data[x], theta[x], robot_position)
		data_to_return[0,x] = point[0]
		data_to_return[1,x] = point[1]
	
	return data_to_return

def draw_plot(linesOfLines, color_of_line, name, data_set_number ):
	lc = mc.LineCollection(linesOfLines, colors=color_of_line, linewidths=2)

	ax = pl.subplots()

	data_to_plot = open_raw_data_for_plot(name, data_set_number)

	ax = plt.subplot(1, 1, 1)
	ax.scatter(data_to_plot[0,:],data_to_plot[1,:],s=1)

	ax.add_collection(lc)

	ax.axis('equal')
	ax.margins(0.1)

	plt.show()

def pol_to_car(r, theta, robot_position):
	x = r * cos(theta + robot_position[2]) + robot_position[0]
	y = r * sin(theta + robot_position[2]) + robot_position[1]
	point = np.matrix([[x],[y]])
	return point

# S should be lower than D * 2 + 1!!!
def RANSAC(data, samples, robot_position, number_of_iteration, chosen_angle, number_of_randomly_chose_sample, distance_from_line, number_of_matching_sample):
	samples_left = 512
	list_of_lines = []
	list_of_lidar_angle = []
	random_index_tab = []

	x = np.arange(0,512)
	theta = (np.pi/512 )*x 

	point_lidar = np.zeros((2,512))

	for x in np.arange(0,512):
		point = pol_to_car(data[x], theta[x], robot_position)
		point_lidar[0,x] = point[0]
		point_lidar[1,x] = point[1]
	

	while number_of_iteration > 0 and samples_left > 2 * chosen_angle + 1:
		number_of_iteration = number_of_iteration - 1

		random_index = random.randint(chosen_angle,511 - chosen_angle)
		random_index_tab.append(random_index)

		rangeIndexes = range(random_index-chosen_angle, random_index+chosen_angle+1)
		sPointsIndexes = random.sample(rangeIndexes, number_of_randomly_chose_sample)

		goodPoints = []
		for pointNumber in sPointsIndexes:
			if (samples[pointNumber] >= 0 ):
				goodPoints.append(pointNumber)	
		
		if not goodPoints:
			continue

		p = np.polyfit(point_lidar[0,goodPoints],point_lidar[1,goodPoints],1)

		
		goodPoints = []
		counter = 0

		for i in range(0,512):
			d = norm(p[0] * point_lidar[0,i] -1 * point_lidar[1,i] + p[1])/np.sqrt(p[0]**2 + 1)
			if (d < distance_from_line):
				counter = counter + 1
				goodPoints.append(i)

		if counter > number_of_matching_sample:
			p = np.polyfit(point_lidar[0,goodPoints],point_lidar[1,goodPoints],1)

			first_good_points = point_lidar[0, goodPoints[0]]
			second_good_pints = point_lidar[0, goodPoints[-1]]

			pointOne = [first_good_points, p[0] * first_good_points + p[1]]
			pointTwo = [second_good_pints, p[0] * second_good_pints + p[1]]
			list_of_lines.append([pointOne, pointTwo])
			list_of_lidar_angle.append([goodPoints[0], goodPoints[-1]])

			for index in goodPoints:
				samples[index] = -1
				samples_left = samples_left - 1

	return list_of_lines, random_index_tab, list_of_lidar_angle


data_set_number=0
name = "line_localization_1.json"			
data = open_json_robot_data(name, data_set_number)
robot_position = open_json_robot_position(name, data_set_number)

listOfLines,rAll, lidar = RANSAC(data, list(data), robot_position, 500,10,5,0.02,50)

correct_lines = []
lines_h = []
lines_v = []

chosenline = []
flag_v = 0
flag_h = 0

for i in range(len(listOfLines)):
	slope = (listOfLines[i][0][1] - listOfLines[i][1][1])/(listOfLines[i][0][0] - listOfLines[i][1][0])
	if slope < 0:
		lines_h.append( listOfLines[i] )
		if flag_v == 0:
			chosenline.append(i)
			flag_v += 1
	else:
		lines_v.append( listOfLines[i] )
		if flag_h == 0:
			chosenline.append(i)
			flag_h += 1

raw_data = open_json_robot_data(name, data_set_number)
print(lidar[chosenline[0]], lidar[chosenline[1]])

print("index lidar line 1:", raw_data[lidar[chosenline[0]][0]], raw_data[lidar[chosenline[0]][1]])
print("index lidat line 2:", raw_data[lidar[chosenline[1]][0]], raw_data[lidar[chosenline[1]][1]])

def avg_line( lines ):
    x1 = 0
    x2 = 0
    y1 = 0
    y2 = 0

    for i in range(len(lines)):
        x1 = x1 + lines[i][0][0]
        x2 = x2 + lines[i][1][0]
        y1 = y1 + lines[i][0][1]
        y2 = y2 + lines[i][1][1]

    n = len(lines)
    return [[x1/n, y1/n], [x2/n, y2/n]]


correct_lines.append( avg_line(lines_h) )
correct_lines.append( avg_line(lines_v) )



def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
	
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


p1 = np.polyfit(correct_lines[0][0], correct_lines[0][1],1)
p2 = np.polyfit(correct_lines[1][0], correct_lines[1][1],1)


robot_pose=[0,0]
robot_pose[1] = abs(p1[1])/np.sqrt(p1[0]**2 + 1)
robot_pose[0] = abs(p2[1])/np.sqrt(p2[0]**2 + 1)


print("robot_pose line_intersection :", line_intersection(correct_lines[0], correct_lines[1]))
print("robot_pose distance_from_line :", robot_pose)



def euclidDist(p1,p2):
	return np.sqrt((p1[0] -p2[0])**2 + (p1[1] -p2[1])**2)

lengthOfLine1 = euclidDist(correct_lines[0][0], correct_lines[0][1])
lengthOfLine2 = euclidDist(correct_lines[1][0], correct_lines[1][1])
			
axes = [[[0 ,0],[0, lengthOfLine1 ]],[[0,0],[lengthOfLine2, 0]]]

lc = mc.LineCollection(axes, linewidths=2)
ax = pl.subplots()
ax = plt.subplot(1, 1, 1)
ax.add_collection(lc)
ax.axis('equal')
ax.margins(0.1)
ax.scatter(robot_pose[0], robot_pose[1])
plt.show()
			


color_of_line = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
#draw_plot(listOfLines, color_of_line, name, data_set_number)
draw_plot(correct_lines, color_of_line, name, data_set_number)



import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, acos
import json
import sys

def pol2Car(r, theta):
	x = r * cos(theta)
	y = r * sin(theta)
	point = np.matrix([[x],[y]])
	return point
def f(z, *args):
	x,y=z
	x1, y1, x2, y2, d1, d2 = args
	return ((((x1-x)**2)+((y1-y)**2)-d1**2)**2)+((((x2-x)**2)+((y2-y)**2)-d2**2)**2)
    


alpha1 = (np.pi/512 )*lidar[chosenline[1]][0] 
alpha2 = (np.pi/512 )*lidar[chosenline[1]][1] 

d1 = raw_data[lidar[chosenline[1]][0]]
d2 = raw_data[lidar[chosenline[1]][1]]
p1 = pol2Car(d1, alpha1)
p2 = pol2Car(d2, alpha2)

#print "Angles to markers in iteration 0:"
#print "alpha1=%f, alpha2=%f" % (alpha1,alpha2)
#print "d1=%f, d2=%f" % (d1,d2)

d0 = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
#print "d0=%f" % (d0)


areaOfTriangle = d1 * d2 * np.sin(np.abs(alpha1 - alpha2)) / 2
#print "areaOfTriangle=%f" % (areaOfTriangle)

hate = areaOfTriangle * 2 / d0
#print "y = hate=%f" % (hate)
y = hate;

beta = np.arcsin(areaOfTriangle * 2 / (d1 * d0));

x = hate / np.tan(beta);

d2ForRightAngled = np.sqrt(d1**2 + d0**2)
if (d2 > d2ForRightAngled):
	x = -x

print("geometric_aproach :", x[0][0], " ", y[0][0])


gamma = np.arcsin(x / d2)
theta = alpha1 + gamma - np.pi/2;
print("geometric_aproach theta: ", theta[0][0])
