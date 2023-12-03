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
        self.map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.center = int(self.numberOfBox/2)

        self.hit_threshold = 10
        self.miss_threshold = -10
        self.p_hit = 0.95
        self.p_miss = 0.3

        self.sensor_displacement = 0.1



    def printMap(self):
        pl.imshow(self.map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show()

        prob_map = self.computeProbab(self.map)
        pl.imshow(prob_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show()
    


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



    def iterateLidar(self, dataScan, robot_position):
        theta = (pl.pi/512 )*(pl.arange(0,512)-256)
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            sensor_position = sensor_shift( self.sensor_displacement, 0, robot_position)

            pt = pol2Car(dataScan[k], theta[k], sensor_position)
            self.substractPointsOnLine( pt, sensor_position)  

            obstacle_x = int(pt[0]/self.resolution) + self.center
            obstacle_y = int(pt[1]/self.resolution) + self.center

            self.map[obstacle_x][obstacle_y] = self.hit( self.map[obstacle_x][obstacle_y] )

        

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

        start_box = self.box_coord( start[0], start[1])
        end_box = self.box_coord( end[0], end[1])

        for x in self.scan_line_range( start_box, end_box, 0):
            for y in self.scan_line_range( start_box, end_box, 1):
                box_pos = self.map_coord(x,y)
                if ( distancePointToLine( A, B, C, box_pos[0], box_pos[1]) < pl.sqrt(2)/2 * self.resolution):
                    self.map[x][y] = self.miss( self.map[x][y] )
                   


    def computeProbab(self, field):
	    return 1 - (1/(1 + pl.exp(field)))	



    def hit(self, field):
        field = field + pl.log(self.p_hit/(1-self.p_hit))
        if (field > self.hit_threshold):
            field = self.hit_threshold
        return field

	    
	    
    def miss(self, field):
        field = field + pl.log(self.p_miss/(1-self.p_miss))
        if (field < self.miss_threshold):
            field = self.miss_threshold
        return field


    def get_xy(self, scan_data, robot_position):
        x = [] 
        y = []

        sensor_position = pol2Car( self.sensor_displacement, 0, robot_position)

        theta = (pl.pi/512 )*(pl.arange(0,512)-256)  # angle in radians

        for i in range(0, 512):
            x.append( scan_data[i] * cos(-theta[i]+robot_position[2]) + sensor_position[0])
            y.append( scan_data[i] * sin(-theta[i]+robot_position[2]) + sensor_position[1])

        return x, y





def pol2Car(r, theta, pose):
    x = r * cos(-theta+pose[2]) + pose[0]
    y = r * sin(-theta+pose[2]) + pose[1]
    point = pl.matrix([[x],[y]])
    return point



def sensor_shift(r, theta, pose):
    x = r * cos(-theta+pose[2]) + pose[0]
    y = r * sin(-theta+pose[2]) + pose[1]
    return [x, y, pose[2]]









dataScan, dataPose = get_raw_data()
print(dataPose)


Map = map()
Map.iterateLidar(dataScan, dataPose)
Map.printMap()

