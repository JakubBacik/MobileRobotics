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
        self.size = 15
        self.resolution = 0.1 
        self.numberOfBox = int(self.size/self.resolution)
        # self.map = [[0]*self.numberOfBox]*self.numberOfBox
        self.map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.center = int(self.numberOfBox/2)



    def printMap(self):
        pl.imshow(self.map, interpolation="nearest",cmap='Blues')
        pl.colorbar()
        pl.show()
    


    def iSM(mi, xt, zt):
        return log10()
    


    def scan_to_map(self, dataScan, dataPose):
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
        theta = (pl.pi/512 )*(pl.arange(0,512))
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            pt = pol2Car(dataScan[k], theta[k], dataPose)
            self.substractPointsOnLine( pt, dataPose)  

            obstacle_x = int(pt[0]/self.resolution) + self.center
            obstacle_y = int(pt[1]/self.resolution) + self.center

            # self.map[obstacle_x][obstacle_y] += 0.1   
            #print("X: ", obstacle_x, " Y: ",obstacle_y, " X_r: ",pt[0], " Y_r: ", pt[1], " T: ", theta[k])
            self.map[obstacle_x][obstacle_y] = 2

        

    def box_coord(self, x, y ):
        box_x = int( x / self.resolution) + self.center
        box_y = int( y / self.resolution) + self.center
        return pl.array([box_x, box_y])



    def map_coord(self, x, y ):
        real_x = (x - self.center) * self.resolution
        real_y = (y - self.center) * self.resolution
        return pl.array([real_x, real_y])



    def scan_line_range(self, start, end, coord):
        scan_range = 0

        if (start[coord] < end[coord]):
            scan_range = range(start[coord], end[coord])
        elif (start[coord] > end[coord]):
            scan_range = range(start[coord], end[coord], -1)
        else:
            scan_range = [start[coord]]

        return scan_range



    def substractPointsOnLine(self, start, end):
        A = end[1] - start[1]
        B = start[0] - end[0]
        C = end[0] * start[1] - start[0] * end[1]

        boxes = []

        start_box = self.box_coord( start[0], start[1])
        end_box = self.box_coord( end[0], end[1])

        for x in self.scan_line_range( start_box, end_box, 0):
            for y in self.scan_line_range( start_box, end_box, 1):
                box_pos = self.map_coord(x,y)
                if ( distancePointToLine( A, B, C, box_pos[0], box_pos[1]) < pl.sqrt(2)/2 * self.resolution):
                    boxes.append([x,y])
                    self.map[x][y] -= 0.1
                   





def pol2Car(r, theta, pose):
    x = r * cos(theta-pose[2]) + pose[0]
    y = r * sin(theta-pose[2]) + pose[1]
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

#pl.plot(x, y, 'r.', markersize = 1)#, color='green')
#pl.xlabel('x')
#pl.ylabel('y')
#pl.axis('equal')
#pl.show()

Map = map()
# Map.something(dataScan, dataPose)
Map.iterateLidar(dataScan, dataPose)
Map.printMap()

