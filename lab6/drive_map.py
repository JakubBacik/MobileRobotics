import json
import pylab as pl
import argparse
from math import cos, sin, log10
import sys
import numpy as np
import codecs


class drive_map:
    
    
    def __init__(self):
        self.size = 15
        self.resolutionSmall = 0.1
        self.numberOfBoxSmall = int(self.size/self.resolutionSmall)

        self.filter_size = 5
        
        self.resolution = self.resolutionSmall * self.filter_size
        self.numberOfBox = int(self.numberOfBoxSmall/self.filter_size)

        self.prob_map =  pl.zeros((self.numberOfBoxSmall, self.numberOfBoxSmall))
        self.center = int(self.numberOfBox/2)
        
        self.obstacle_threshold = 0.9
        self.start_position = [9, 13]
        self.end_position = [15, 25]

        self.drive_map = -pl.ones((self.numberOfBox, self.numberOfBox))
        self.path_map = -pl.ones((self.numberOfBox, self.numberOfBox))

        self.path = []

    def print_drive_map(self):
        self.drive_show = pl.imshow( self.drive_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.colorbar()
        pl.show()

    def print_path_map(self):
        self.path_show = pl.imshow( self.path_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.colorbar()
        pl.show( block=False )
        pl.pause( 0.1 )


    def pixellate_map(self):    
        for x in range(0, self.numberOfBoxSmall, self.filter_size):
            for y in range(0, self.numberOfBoxSmall, self.filter_size):
                for x_s in range(x, x+self.filter_size):
                    for y_s in range( y, y+self.filter_size):
                        if self.prob_map[x_s][y_s] > self.obstacle_threshold:
                            self.drive_map[int(x/self.filter_size)][int(y/self.filter_size)] = 123456

    def calculate_dists(self):
        #         up  right  down   left
        diff = [[0,1],[1,0],[0,-1],[-1,0]]
        self.path_map[self.end_position[1]][self.end_position[0]] = 0
        for d in range(0, 2*self.numberOfBox):
            for x in range(0, self.numberOfBox):
                for y in range(0, self.numberOfBox):
                    if( self.drive_map[x][y] == 123456):
                        self.path_map[x][y] = 50

                    elif( self.path_map[x][y] == d):
                        for dir in range(0,4):
                            x_s = x + diff[dir][0]
                            y_s = y + diff[dir][1]
                            
                            if( x_s < self.numberOfBox and x_s >= 0 and y_s < self.numberOfBox and y_s >= 0 and self.path_map[x_s][y_s] == -1):
                                self.path_map[x_s][y_s] = d + 1
                

    def path_finding(self):
         #         up  right  down   left
        diff = [[0,1],[1,0],[0,-1],[-1,0]]

        x = self.start_position[1]
        y = self.start_position[0]

        x_p = 0
        y_p = 0
        x_path = []
        y_path = []

        self.start_value = self.path_map[x][y]

        for d in range(0, 2*self.numberOfBox):
            x_path.append(x)
            y_path.append(y)
            if(self.end_position[1] == x and self.end_position[0] == y):
                break
            for dir in range(0,4):
                x_s = x + diff[dir][0]
                y_s = y + diff[dir][1]

                if( x_s <= self.numberOfBox and x_s >= 0 and y_s <= self.numberOfBox and y_s >= 0 and self.path_map[x_s][y_s] !=123456):
                    if( self.start_value >= self.path_map[x_s][y_s]):
                        x_p = x_s
                        y_p = y_s
                        self.start_value = self.path_map[x_s][y_s]

            x = x_p
            y = y_p
        
        self.path = [x_path, y_path]    
                            
                            

                        



def read_file():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="name of the json data file")
    # parser.add_argument("start_x", help="start x")
    # parser.add_argument("start_y", help="start y")
    # parser.add_argument("end_x", help="end x")
    # parser.add_argument("end_y", help="end y")
    args = parser.parse_args()

    obj_text = codecs.open( args.filename, 'r', encoding='utf-8').read()
    b_new = json.loads(obj_text)
    data = np.array(b_new)

    return data

map = drive_map()
map.prob_map = read_file()
# pl.imshow( map.prob_map, interpolation="nearest", cmap='Blues', origin='lower')
pl.figure()
map.pixellate_map()
#map.print_drive_map()

map.calculate_dists()
map.print_path_map()
map.path_finding()

print(map.path[0])
print(map.path[1])

pl.plot(map.path[1], map.path[0],'r-',linewidth=1)
pl.plot(map.start_position[0], map.start_position[1],'go',linewidth=1)
pl.plot(map.end_position[0], map.end_position[1],'go',linewidth=1)
pl.text(map.start_position[0]-0.5, map.start_position[1]+0.5, 'Start', fontsize=5)
pl.text(map.end_position[0]-0.5, map.end_position[1]+0.5, 'End', fontsize=5)
pl.show( )
pl.pause( 1.0 )

