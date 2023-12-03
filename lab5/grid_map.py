import json
import pylab as pl
import argparse
from math import cos, sin, log10
import sys



def get_raw_data():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="name of the json data file")
    parser.add_argument("number", help="database set number")
    args = parser.parse_args()

    json_data = open(args.filename)
    data = json.load(json_data)
    dataset = int(args.number)

    return data[dataset]["scan"], data[dataset]["pose"]

def distancePointToLine(A, B, C, xPoint, yPoint):
    return pl.absolute(A * xPoint + B * yPoint + C) / pl.sqrt(A*A + B*B)

class map:
    
    def __init__(self):
        self.size = 10
        self.resolution = 0.1 
        self.numberOfBox = int(self.size/self.resolution)
        # self.map = [[0]*self.numberOfBox]*self.numberOfBox
        self.map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.center = int(self.numberOfBox/2)



    def printMap(self):
        pl.imshow(self.map, interpolation="nearest",cmap='autumn')
        pl.colorbar()
        pl.show()
    


    def iSM(mi, xt, zt):
        return log10()
    


    def something(self, dataScan, dataPose):
        theta = (pl.pi/512 )*(pl.arange(0,512)-256)  # angle in radian

        robot_x = int( dataPose[0]/self.resolution) + self.center
        robot_y = int(dataPose[1]/self.resolution) + self.center
        self.map[robot_x][robot_y] = -1
        
        for i in range(self.numberOfBox):
            for j in range(self.numberOfBox):
                for k in range(len(dataScan)):
                    if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                        continue

                    pt = pol2Car(dataScan[k], theta[k], dataPose)
                    if pt[0] > i*self.resolution and pt[0] < (i+1)*self.resolution and pt[1] > j*self.resolution and pt[1] < (j+1)*self.resolution:
                        self.map[i][j] += 0.1


    def iterateLidar(self, dataScan, dataPose):
        theta = (pl.pi/512 )*(pl.arange(0,512)-256)
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            pt = pol2Car(dataScan[k], theta[k], dataPose)
            self.substractPointsOnLine( dataPose, pt)  

            obstacle_x = int(pt[0]/self.resolution) + self.center
            obstacle_y = int(pt[1]/self.resolution) + self.center

            # self.map[obstacle_x][obstacle_y] += 0.1   
            print(obstacle_x, " ",obstacle_y, " ", theta[k])

        
    # def substractPointsOnLine(self, robot_p, obstacle_p):
    #     points = []
    #     robot_x = int(robot_p[0]/self.resolution) + self.center
    #     robot_y = int(robot_p[1]/self.resolution) + self.center
    #     obstacle_x = int(obstacle_p[0]/self.resolution) + self.center
    #     obstacle_y = int(obstacle_p[1]/self.resolution) + self.center

    #     dx = robot_p[1] - obstacle_p[1]
    #     dy = obstacle_p[0] - robot_p[0] 
    #     C = robot_p[0] * obstacle_p[1] - obstacle_p[0] * robot_p[1]


    #     for x in range(robot_x, obstacle_x):
    #         for y in range(robot_y, obstacle_y):
    #             if (distancePointToLine(dx, dy, C, x, y) < pl.sqrt(2)/2 * self.resolution):
    #                 self.map[x][y] -= 0.1

    def substractPointsOnLine(self, startPoint, endPoint):
        A = endPoint[1] - startPoint[1]
        B = startPoint[0] - endPoint[0]
        C = endPoint[0] * startPoint[1] - startPoint[0] * endPoint[1]
        boxes = []
        robot_x = int(startPoint[0]/self.resolution) + self.center
        robot_y = int(startPoint[1]/self.resolution) + self.center
        startPointInt = (robot_x, robot_y)

        obstacle_x = int(endPoint[0]/self.resolution) + self.center
        obstacle_y = int(endPoint[1]/self.resolution) + self.center

        endPointInt = (obstacle_x, obstacle_y)
        rangeForX = 0;
        if (startPointInt[0] < endPointInt[0]):
            rangeForX = range(startPointInt[0], endPointInt[0])
        elif (startPointInt[0] > endPointInt[0]):
            rangeForX = range(startPointInt[0], endPointInt[0], -1)
        else:
            rangeForX = [startPointInt[0]]
        rangeForY = 0;
        if (startPointInt[1] < endPointInt[1]):
            rangeForY = range(startPointInt[1], endPointInt[1])
        elif (startPointInt[1] > endPointInt[1]):
            rangeForY = range(startPointInt[1], endPointInt[1], -1)
        else:
            rangeForY = [startPointInt[1]]
            
        for x in rangeForX:
            for y in rangeForY:
                globPos = globalPosOfBox([x,y])
                if (distancePointToLine(A,B,C,globPos[0],globPos[1]) < np.sqrt(2)/2 * boxSize):
                    boxes.append([x,y])
                    self.map[x][y] -= 0.1
                   




def pol2Car(r, theta, pose):
    x = r * cos(-theta+pose[2]) + pose[0]
    y = r * sin(-theta+pose[2]) + pose[1]
    point = pl.matrix([[x],[y]])
    return point

def get_xy(scan_data, robot_position):
    x = [] 
    y = []   

    theta = (pl.pi/512 )*(pl.arange(0,512)-256)  # angle in radians

    for i in range(0, 512):
        x.append(scan_data[i] * cos(-theta[i]+robot_position[2]) + robot_position[0])
        y.append(scan_data[i] * sin(-theta[i]+robot_position[2]) + robot_position[1])

    return x, y

dataScan, dataPose = get_raw_data()
print(dataPose)

x, y = get_xy(dataScan, dataPose)

pl.plot(x, y, 'r.', markersize = 1)#, color='green')
pl.xlabel('x')
pl.ylabel('y')
pl.axis('equal')
pl.show()

Map = map()
# Map.something(dataScan, dataPose)
Map.iterateLidar(dataScan, dataPose)
Map.printMap()

