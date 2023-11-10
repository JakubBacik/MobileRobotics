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
		if math.isinf(data[i]):
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

			for index in goodPoints:
				samples[index] = -1
				samples_left = samples_left - 1

	return list_of_lines, random_index_tab

data_set_number=3
name = "line_localization_1.json"			
data = open_json_robot_data(name, data_set_number)
robot_position = open_json_robot_position(name, data_set_number)

listOfLines,rAll = RANSAC(data, list(data), robot_position, 500,10,5,0.005,50)

radius = 0.1
correct_lines = []
other_lines = []
baseline = listOfLines[0]
correct_lines.append(baseline)

for index in range(len(listOfLines)):
    if abs(baseline[0][0] - listOfLines[index][0][0]) > radius or\
        abs(baseline[0][1] - listOfLines[index][0][1]) > radius or\
        abs(baseline[1][0] - listOfLines[index][1][0]) > radius or\
        abs(baseline[1][1] - listOfLines[index][1][1]) > radius:
        other_lines.append(listOfLines[index])
		
correct_lines.append(other_lines[0])


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
robot_pose[0] = abs(p1[1])/np.sqrt(p1[0]**2 + 1)
robot_pose[1] = abs(p2[1])/np.sqrt(p2[0]**2 + 1)

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
ax.scatter(abs(robot_pose[0]), abs(robot_pose[1]))
plt.show()
			


color_of_line = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)])
#draw_plot(listOfLines, color_of_line, name, data_set_number)
draw_plot(correct_lines, color_of_line, name, data_set_number)