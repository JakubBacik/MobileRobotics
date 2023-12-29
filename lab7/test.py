from matplotlib import pyplot as plt
import numpy as np
import math

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def map_coord(x, y):
    number_of_box_before_reducing = int(25/0.1)
    center_before_reducing = int(number_of_box_before_reducing/2)
    real_x = (x*5 - center_before_reducing) * 0.1
    real_y = (y*5 - center_before_reducing) * 0.1
    return [real_x, real_y, 0]
 

lil = [[29, 30, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40], [25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40]]


def box_coord(x, y):
    number_of_box_before_reducing = int(25/0.1)
    center_before_reducing = int(number_of_box_before_reducing/2)
    box_x = int( x / 0.1) + center_before_reducing
    box_y = int( y / 0.1) + center_before_reducing
    return [int(box_x/5), int(box_y/5)]

min_angle = -2.356100082397461
max_angle = 2.0932999382019043
angle_increment = 0.006144199054688215


theta = np.arange(min_angle, max_angle, angle_increment)
print(len(theta))
print(theta[0], theta[-1])

theta = np.pi/528 * np.arange(0,528) - np.pi/2
print(theta[0], theta[-1])