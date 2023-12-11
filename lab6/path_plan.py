import json
import pylab as pl
import argparse
from math import cos, sin, log10
import sys
import numpy as np




class map:
    
    
    def __init__(self):
        self.size = 15
        self.resolution = 0.1
        self.numberOfBox = int(self.size/self.resolution)
        self.map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.prob_map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.center = int(self.numberOfBox/2)
        self.filter_size = 5
        self.obstacle_threshold = 0.33

        self.drive_map =  -pl.ones((int(self.numberOfBox/self.filter_size), int(self.numberOfBox/self.filter_size)))

        self.hit_threshold = 10
        self.miss_threshold = -10
        self.p_hit = 0.95
        self.p_miss = 0.3

        self.sensor_displacement = 0.05
        self.pathX = []
        self.pathY = []



    def printMap(self, pose):
        boxCordPos = self.box_coord(pose[0], pose[1])
        self.pathX.append(boxCordPos[0])
        self.pathY.append(boxCordPos[1])    
        self.show = pl.imshow( self.prob_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.colorbar()
        pl.show( block=False )
        pl.pause( 0.1 )

    def print_drive_map(self):
        self.drive_show = pl.imshow( self.drive_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.colorbar()
        pl.show()



    def update(self, pose):
        boxCordPos = self.box_coord(pose[0], pose[1])
        self.pathX.append(boxCordPos[0])
        self.pathY.append(boxCordPos[1])
        self.show.set_data(self.prob_map)
        pl.plot(self.pathX, self.pathY,'r-',linewidth=1)
        pl.show( block=False )
        pl.pause( 0.1 )
        
#        self.fig.hold()



    def iterateLidar(self, dataScan, robot_position):
        theta = (pl.pi/512 )*pl.arange(0,512) - pl.pi/2
        robot_position[2] = (robot_position[2]/180) * pl.pi
        sensor_position = sensor_shift( self.sensor_displacement, 0, robot_position)
        print(sensor_position)
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            pt = pol2Car(dataScan[k], theta[k], sensor_position)
            self.substractPointsOnLine( pt, sensor_position)  

            obstacle_x = int(pt[0]/self.resolution) + self.center
            obstacle_y = int(pt[1]/self.resolution) + self.center

            self.map[obstacle_y][obstacle_x] = self.hit( self.map[obstacle_y][obstacle_x] )

        self.prob_map = self.computeProbab(self.map)

        

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
                    self.map[y][x] = self.miss( self.map[y][x] )
                   


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


    def pixellate_map(self):      
        for x in range(0, self.numberOfBox, self.filter_size):
            for y in range(0, self.numberOfBox, self.filter_size):
                for x_s in range(x, x+self.filter_size):
                    for y_s in range( y, y+self.filter_size):
                        if self.prob_map[x_s][y_s] > self.obstacle_threshold:
                            self.drive_map[int(x/self.filter_size)][int(y/self.filter_size)] = 123456
                        

                


def pol2Car(r, theta, pose):
    x = r * cos(theta+pose[2]) + pose[0]
    y = r * sin(theta+pose[2]) + pose[1]
    point = pl.matrix([[x],[y]])
    return point



def sensor_shift(r, theta, pose):
    x = r * cos(theta+pose[2]) + pose[0]
    y = r * sin(theta+pose[2]) + pose[1]
    return [x, y, pose[2]]




def get_raw_data():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="name of the json data file")
    parser.add_argument("number", help="database set number")
    args = parser.parse_args()

    json_data = open(args.filename)
    data = json.load(json_data)
    dataset = int(args.number)

    return data[dataset]["scan"], data[dataset]["pose"]



def get_raw_data_bulk():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="name of the json data file")
    args = parser.parse_args()

    json_data = open(args.filename)
    data = json.load(json_data)

    return data, len(data)




def distancePointToLine(A, B, C, xPoint, yPoint):
    return pl.absolute(A * xPoint + B * yPoint + C) / pl.sqrt(A*A + B*B)





data, size = get_raw_data_bulk()

Map = map()
Map.iterateLidar( data[0]["scan"], data[0]["pose"])
Map.printMap(data[0]["pose"])

for dataset in range(1, size):
    print("dataset: ")
    print(data[dataset]["pose"])
    
    Map.iterateLidar( data[dataset]["scan"], data[dataset]["pose"])
    Map.update(data[dataset]["pose"])


Map.printMap(data[0]["pose"])


pl.figure()
# Map.pixellate_map()
# Map.print_drive_map()




















# Write map to file json
# b = Map.prob_map.tolist() 
# file_path = "map21.json" 
# json.dump(b, codecs.open(file_path, 'w', encoding='utf-8'), 
#           separators=(',', ':'), 
#           sort_keys=True, 
#           indent=4) 
