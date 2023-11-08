import json
from scipy import optimize
from matplotlib import collections  as mc
import pylab as pl
<<<<<<< HEAD
import argparse
import math



epsilon = 0.15
multistep = 5

=======
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
>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b

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

<<<<<<< HEAD




def line_fitting(x, y, start):
=======
def show_plot():
    plt.show()

def curve_fitting(x, y, start, scan_data):
>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b
    if start+2 >= len(x):
        return len(x)-1
    
    alpha_o = [0,0]
    first_iter = True
    multipoint = True
#    tmp = 0

    for i in range(start+2, len(x)):
<<<<<<< HEAD
        if scan_data[i] > 50:
            #print("NaNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNn")
            return i
        
        else:
            if multipoint:
                multi_i = i
                for j in range( i, min(len(x), i+multistep) ):
                    if scan_data[i] > 50:
                        i = j-1
                        break

#            tmp = tmp+1
            alpha = optimize.curve_fit(func, xdata = x[start:i], ydata = y[start:i])[0]

            if first_iter:
=======
        if scan_data[i] < 100:
            tmp= tmp+1
            alpha = optimize.curve_fit(func, xdata = x[start:i], ydata = y[start:i])[0]

            #alpha = np.polyfit(x[start:i], y[start:i], 1)

            if i == start+2:
>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b
                alpha_o = alpha
                first_iter = False
            
<<<<<<< HEAD
            if abs(alpha[0]-alpha_o[0]) > epsilon or abs(alpha[1]-alpha_o[1]) > epsilon:
                if multipoint:
                    i = multi_i
                    multipoint = False
                else:
                    #print("Line break")
                    return i
#            else:
#                if not multipoint:
#                    multipoint = True

#            if tmp%5==0:
#                alpha_o = alpha
=======
            if abs(alpha[0]-alpha_o[0]) > epsilonA and abs(alpha[1]-alpha_o[1]) > epsilonB:
                return i
            
            if tmp%7==0:
                alpha_o = alpha

        else:
            return i
>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b
    
    return i


<<<<<<< HEAD



parser = argparse.ArgumentParser()
parser.add_argument("filename", help="name of the json data file")
args = parser.parse_args()

##reading data from file
json_data = open(args.filename)
data = json.load(json_data)

x = pl.arange(0,512)
theta = (pl.pi/512 )*(x-256)  # angle in radians

scan_data=data[0]["scan"]
robot_position=data[0]["pose"]

x = [] 
y = []   

for i in range(0, 512):
    x.append(scan_data[i] * math.cos(-theta[i]+robot_position[2]) + robot_position[0])
    y.append(scan_data[i] * math.sin(-theta[i]+robot_position[2]) + robot_position[1])


idx = 0
lines = []
while idx < len(x)-1:
    line_tmp = []
    while scan_data[idx] > 50:
        if idx < len(x)-1:
            idx += 1
        else:
            break
=======
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

>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b

    line_tmp.append( (x[idx],y[idx]) )
    idx = line_fitting(x,y,idx)
    line_tmp.append( (x[idx-1], y[idx-1]) )

<<<<<<< HEAD
    lines.append(line_tmp)



#Detected lines plot
lc = mc.LineCollection( lines, linewidths=2)
fig, ax = pl.subplots()

#pl.style.use('dark_background')
pl.get_current_fig_manager().full_screen_toggle()

ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

#Data points plot
#pl.plot(x, y, 'r.', markersize = 1)
pl.xlabel('x')
pl.ylabel('y')

pl.show()
=======
epsilonA = 0.174
epsilonB = 0.174

x, y, scan_data = open_json('line_detection_1.json')
lines = line_detection_alg(x,y, scan_data)

plot_from_line(lines)


show_plot()
>>>>>>> c8e3bb669c8567270029310d3c3633418a8f5b8b
