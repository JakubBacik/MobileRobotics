import json
from scipy import optimize
from matplotlib import collections  as mc
import pylab as pl
##reading data from the file
json_data = open('line_detection_1.json')
data = json.load(json_data)

### plot in polar system
import matplotlib.pyplot as plt
import numpy as np
import math

x = np.arange(0,512)
theta = (np.pi/512 )*(x-256)  # angle in radians

scan_data=data[0]["scan"]
robot_position=data[0]["pose"]
epsilon = 0.15
x = [] 
y = []   

for i in range(0, 512):
    x.append(scan_data[i] * math.cos(-theta[i]+robot_position[2]) + robot_position[0])
    y.append(scan_data[i] * math.sin(-theta[i]+robot_position[2]) + robot_position[1])


plt.figure(figsize = (10,8))
plt.plot(x, y, 'b.')
plt.xlabel('x')
plt.ylabel('y')


def func(x, a, b):
    y = a*x + b
    return y

def curve_fitting(x, y, start):
    if start+2 >= len(x):
        return len(x)-1
    
    alpha_o = [0,0]
    tmp = 0
    for i in range(start+2, len(x)):
        if scan_data[i] < 50:
            tmp= tmp+1
            alpha = optimize.curve_fit(func, xdata = x[start:i], ydata = y[start:i])[0]

            if i == start+2:
                alpha_o = alpha
            
            if abs(alpha[0]-alpha_o[0]) > epsilon and abs(alpha[1]-alpha_o[1]) > epsilon:
                print("Line break")
                return i
            if tmp%5==0:
                alpha_o = alpha

        else:
            print("NaNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNn")
            return i
    
    return i

idx = 0
toPlot = []
while idx < len(x)-1:
    tmp = []
    while scan_data[idx] > 50:
        idx += 1
    tmp.append((x[idx],y[idx]))
    idx = curve_fitting(x,y,idx)
    tmp.append((x[idx-1], y[idx-1]))

    toPlot.append(tmp)



# plt.figure(figsize = (10,8))
# plt.plot(x, y, 'b.')
# plt.plot(x, alpha[0]*np.array(x) + alpha[1], 'r')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.show()

lc = mc.LineCollection(toPlot, linewidths=2)

fig, ax = pl.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)

pl.show()