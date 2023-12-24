import json
import pylab as pl
import argparse
from math import cos, sin, log10, copysign
import sys


class grid_map:
    
    def __init__(self):
        self.size = 50
        self.resolution = 0.05
        self.numberOfBox = int(self.size/self.resolution)
        self.map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.prob_map =  pl.ones((self.numberOfBox, self.numberOfBox))/2
        self.center = int(self.numberOfBox/2)

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
        pl.show( block=False )
        pl.pause( 0.1 )



    def update(self, pose):
        boxCordPos = self.box_coord(pose[0], pose[1])
        self.pathX.append(boxCordPos[0])
        self.pathY.append(boxCordPos[1])
        self.show.set_data(self.prob_map)
        pl.plot(self.pathX, self.pathY,'r-',linewidth=1)
        pl.show( block=False )
        pl.pause( 0.1 )
        


    def iterateLidar(self, dataScan, robot_position):
        theta = (pl.pi/512 )*pl.arange(0,512) - pl.pi/2
        robot_position[2] = (robot_position[2]/180) * pl.pi
        sensor_position = sensor_shift( self.sensor_displacement, 0, robot_position)
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            pt = pol2Car(dataScan[k], theta[k], sensor_position)
#            self.substractPointsOnLine( pt, sensor_position)  
            self.ray_trace( pt, sensor_position)

            obstacle = self.box_coord(pt[0], pt[1])

            self.map[obstacle[1]][obstacle[0]] = self.hit( self.map[obstacle[1]][obstacle[0]] )

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

#        hits = 0
#        boxes_analyzed = 0
        
        for x in self.scan_line_range( start_box, end_box, 0):
            for y in self.scan_line_range( start_box, end_box, 1):
                box_pos = self.map_coord(x,y)
#                boxes_analyzed = boxes_analyzed + 1
                if ( distancePointToLine( A, B, C, box_pos[0], box_pos[1]) < pl.sqrt(2)/2 * self.resolution):
                    self.map[y][x] = self.miss( self.map[y][x] )
#                    hits = hits + 1

#        print( f'Total analyzed: {boxes_analyzed}  Hits: {hits} dx: {dx}  dy: {dy}')

    def ray_trace(self, end, start):
        A = end[1] - start[1]
        B = start[0] - end[0]
        C = end[0] * start[1] - start[0] * end[1]

        dist_den = pl.sqrt(A*A + B*B)

        start_box = self.box_coord( start[0], start[1])
        end_box = self.box_coord( end[0], end[1])

#        hits = 0
#        boxes_analyzed = 0
        threshold = pl.sqrt(2)/2 * self.resolution
#        threshold = 1/2 * self.resolution
        run = 1

        x = start_box[0]
        y = start_box[1]

        dx = end_box[0] - x
        dy = end_box[1] - y


        if( abs(dx) >= abs(dy) ):
            for x in self.scan_line_range( start_box, end_box, 0):
                center_hit = 0
                r_hit = 0
                l_hit = 0

                x_r = x
                y_r = y + 1
                x_l = x
                y_l = y - 1
                
                real_x = (x - self.center) * self.resolution
                real_y = (y - self.center) * self.resolution
                center_hit = (abs( A*real_x + B*real_y + C) / dist_den) < threshold
#                boxes_analyzed = boxes_analyzed + 1

                if( x_r < self.numberOfBox and x_r >= 0 and y_r < self.numberOfBox and y_r >= 0 ):
                    real_x_r = (x_r - self.center) * self.resolution
                    real_y_r = (y_r - self.center) * self.resolution
                    r_hit = (abs( A*real_x_r + B*real_y_r + C) / dist_den) < threshold
#                    boxes_analyzed = boxes_analyzed + 1
                    if( r_hit ):
                        self.map[y_r][x_r] = self.miss( self.map[y_r][x_r] )
#                       hits = hits + 1

                if( x_l < self.numberOfBox and x_l >= 0 and y_l < self.numberOfBox and y_l >= 0 ):
                    real_x_l = (x_l - self.center) * self.resolution
                    real_y_l = (y_l - self.center) * self.resolution
                    l_hit = (abs( A*real_x_l + B*real_y_l + C) / dist_den) < threshold
# boxes_analyzed = boxes_analyzed + 1
                    if( l_hit ):
                        self.map[y_l][x_l] = self.miss( self.map[y_l][x_l] )
    #                    hits = hits + 1

                if( center_hit ):
                    self.map[y][x] = self.miss( self.map[y][x] )
#                    hits = hits + 1
                elif( r_hit ):
                    y = y_r
                elif( l_hit ):
                    y = y_l


        else:
            for y in self.scan_line_range( start_box, end_box, 1):
                center_hit = 0
                r_hit = 0
                l_hit = 0
                
                x_r = x + 1
                y_r = y
                x_l = x - 1
                y_l = y

                real_x = (x - self.center) * self.resolution
                real_y = (y - self.center) * self.resolution
                center_hit = (abs( A*real_x + B*real_y + C) / dist_den) < threshold
#                boxes_analyzed = boxes_analyzed + 1

                if( x_r < self.numberOfBox and x_r >= 0 and y_r < self.numberOfBox and y_r >= 0 ):
                    real_x_r = (x_r - self.center) * self.resolution
                    real_y_r = (y_r - self.center) * self.resolution
                    r_hit = (abs( A*real_x_r + B*real_y_r + C) / dist_den) < threshold
#                    boxes_analyzed = boxes_analyzed + 1
                    if( r_hit ):
                        self.map[y_r][x_r] = self.miss( self.map[y_r][x_r] )
#                       hits = hits + 1

                if( x_l < self.numberOfBox and x_l >= 0 and y_l < self.numberOfBox and y_l >= 0 ):
                    real_x_l = (x_l - self.center) * self.resolution
                    real_y_l = (y_l - self.center) * self.resolution
                    l_hit = (abs( A*real_x_l + B*real_y_l + C) / dist_den) < threshold
# boxes_analyzed = boxes_analyzed + 1
                    if( l_hit ):
                        self.map[y_l][x_l] = self.miss( self.map[y_l][x_l] )
    #                    hits = hits + 1

                if( center_hit ):
                    self.map[y][x] = self.miss( self.map[y][x] )
#                    hits = hits + 1
                elif( r_hit ):
                    x = x_r
                elif( l_hit ):
                    x = x_l


#        print( f'Total analyzed: {boxes_analyzed}  Hits: {hits} dx: {dx}  dy: {dy}')


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
     return abs(A * xPoint + B * yPoint + C) / pl.sqrt(A*A + B*B)





# data, size = get_raw_data_bulk()

# Map = map()
# Map.iterateLidar( data[0]["scan"], data[0]["pose"])
# Map.printMap(data[0]["pose"])

# for dataset in range(1, size):
# #    input()
#     print(f'dataset: {dataset+1}/{size}')
#     print('Pose: ', data[dataset]["pose"])
    
#     Map.iterateLidar( data[dataset]["scan"], data[dataset]["pose"])
#     Map.update(data[dataset]["pose"])

# pl.show()

