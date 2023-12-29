import numpy as np
import pylab as pl
from math import cos, sin, log10

class map:
    def __init__(self):
        self.size = 25
        self.resolution = 0.05

        self.number_of_box = int(self.size/self.resolution)

        self.grid_map = pl.ones((self.number_of_box, self.number_of_box))/2
        self.probabilistic_map = pl.ones((self.number_of_box, self.number_of_box))/2
        self.center_of_map = int(self.number_of_box/2)

        self.hit_threshold = 10
        self.miss_threshold = -10
        self.p_hit = 0.95
        self.p_miss = 0.3

        self.sensor_displacement = 0.05
        self.path_x = []
        self.path_y = []

    def show_probabilistic_map(self, pose):
        box_coord_pos = self.box_coord(pose[0], pose[1])

        self.path_x.append(box_coord_pos[0])
        self.path_y.append(box_coord_pos[1])

        self.show = pl.imshow(self.probabilistic_map, interpolation="nearest", cmap='Blues', origin='lower')    
        pl.show(block=False)
        pl.pause(0.1)

    def update_probabilistic_map(self, pose):
        box_coord_pos = self.box_coord(pose[0], pose[1])

        self.path_x.append(box_coord_pos[0])
        self.path_y.append(box_coord_pos[1])

        self.show.set_data(self.probabilistic_map)
        pl.plot(self.path_x, self.path_y,'r-',linewidth=1)
        pl.show(block=False)
        pl.pause(0.1)