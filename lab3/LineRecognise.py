import json
from scipy import optimize
from matplotlib import collections  as mc
import pylab as pl
import matplotlib.pyplot as plt
import numpy as np
import math

def func(x, a, b):
    y = a*x + b
    return y

def open_json(name):
    json_data = open(name)
    data = json.load(json_data)

    x = np.arange(0,512)
    theta = (np.pi/512 )*(x-256)  

    scan_data=data[0]["scan"]
    robot_position=data[0]["pose"]
   
    x = [] 
    y = []   

    for i in range(0, 512):
        x.append(scan_data[i] * math.cos(-theta[i]+robot_position[2]) + robot_position[0])
        y.append(scan_data[i] * math.sin(-theta[i]+robot_position[2]) + robot_position[1])
    
    return x, y, scan_data

def plot_raw_data(x, y):
    plt.figure(figsize = (10,8))
    plt.plot(x, y, 'b.')
    plt.xlabel('x')
    plt.ylabel('y')

def plot_from_line(line_tab):
    lc = mc.LineCollection(line_tab, linewidths=2)

    fig, ax = pl.subplots()
    ax.add_collection(lc)
    ax.autoscale()
    ax.margins(0.1)

def show_plot():
    plt.show()

def curve_fitting(x, y, start, scan_data):
    if start+2 >= len(x):
        return len(x)-1
    
    alpha_o = [0,0]
    tmp = 0
    for i in range(start+2, len(x)):
        if scan_data[i] < 100:
            tmp= tmp+1
            alpha = optimize.curve_fit(func, xdata = x[start:i], ydata = y[start:i])[0]

            #alpha = np.polyfit(x[start:i], y[start:i], 1)

            if i == start+2:
                alpha_o = alpha
            
            if abs(alpha[0]-alpha_o[0]) > epsilonA and abs(alpha[1]-alpha_o[1]) > epsilonB:
                return i
            
            if tmp%7==0:
                alpha_o = alpha

        else:
            return i
    
    return i


def line_detection_alg(x,y, scan_data):
    idx = 0
    toPlot = []

    while idx < len(x)-1:
        tmp = []

        while scan_data[idx] > 100:
            idx += 1

        tmp.append((x[idx],y[idx]))
        idx = curve_fitting(x,y,idx, scan_data)
        tmp.append((x[idx-1], y[idx-1]))

        toPlot.append(tmp)

    return toPlot



epsilonA = 0.174
epsilonB = 0.174

x, y, scan_data = open_json('line_detection_1.json')
lines = line_detection_alg(x,y, scan_data)

plot_from_line(lines)


show_plot()