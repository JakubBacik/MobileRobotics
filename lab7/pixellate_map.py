import pylab as pl
import numpy as np
import math

class pixellate_map():
    def __init__(self):
        self.size = 25
        self.resolutionSmall = 0.1
        self.numberOfBoxSmall = int(self.size/self.resolutionSmall)

        self.filter_size = 5
        
        self.resolution = self.resolutionSmall * self.filter_size
        self.numberOfBox = int(self.numberOfBoxSmall/self.filter_size)

        self.prob_map =  pl.zeros((self.numberOfBoxSmall, self.numberOfBoxSmall))
        self.center = int(self.numberOfBox/2)
        
        self.obstacle_threshold = 0.8
        self.start_position = [1, 1]
        self.end_position = [3, 3]
        self.start_angle = 0

        self.drive_map = -pl.ones((self.numberOfBox, self.numberOfBox))
        self.path_map = -pl.ones((self.numberOfBox, self.numberOfBox))

        self.path = []
        self.angle= []

    def set_start_position(self, start_position):
        self.start_position = start_position
    
    def clear_drive_map(self):
        self.path_map = -pl.ones((self.numberOfBox, self.numberOfBox))
        self.path = []

    
    def set_end_position(self, end_position):
        self.end_position = end_position

    def show_data(self, obstacle_map_msg_data):
        text = []
        for i in self.drive_map:
            for j in i:
                if j == 123456.0:
                    text.append(-1)
                else:
                    text.append(int(abs(j)))

        obstacle_map_msg_data.publish_map(text)


    def pixellate_map(self):    
        for x in range(0, self.numberOfBoxSmall, self.filter_size):
            for y in range(0, self.numberOfBoxSmall, self.filter_size): 
                # Initialize a flag to check if there's an obstacle in the current block
                obstacle_in_block = False
                for x_s in range(x, min(x+self.filter_size, self.numberOfBoxSmall)):
                    for y_s in range(y, min(y+self.filter_size, self.numberOfBoxSmall)):
                        if self.prob_map[x_s][y_s] > self.obstacle_threshold:
                            obstacle_in_block = True
                            x1 = x_s // self.filter_size if x_s % self.filter_size < 3 else x_s // self.filter_size + 1
                            y1 = y_s // self.filter_size if y_s % self.filter_size < 3 else y_s // self.filter_size + 1

                            self.drive_map[x1][y1] = 123456
                            break
                    if obstacle_in_block:
                        break
                    

    def calculate_dists(self):
        #         up  right  down   left
        diff = [[0,1],[1,0],[0,-1],[-1,0]]
        self.path_map[self.end_position[1]][self.end_position[0]] = 0

        for d in range(0, 2*self.numberOfBox):
            for x in range(0, self.numberOfBox):
                for y in range(0, self.numberOfBox):
                    if( self.drive_map[x][y] == 123456):
                        self.path_map[x][y] = 500

                    elif( self.path_map[x][y] == d):
                        for dir in range(0,4):
                            x_s = x + diff[dir][0]
                            y_s = y + diff[dir][1]
                            
                            if( x_s < self.numberOfBox and x_s >= 0 and y_s < self.numberOfBox and y_s >= 0 ):
                                if( self.path_map[x_s][y_s] == -1):
                                    self.path_map[x_s][y_s] = d + 1
                

    def path_finding(self):
         #         up  right  down   left
        diff = [[0,1],[1,0],[0,-1],[-1,0]]

        x = self.start_position[1]
        y = self.start_position[0]
        theta = 0

        x_p = 0
        y_p = 0

        x_path = []
        y_path = []
        theta_path = []

        self.start_value = self.path_map[x][y]

        for d in range(0, 2*self.numberOfBox):
            x_path.append(x)
            y_path.append(y)
            theta_path.append(theta)
            
            if(self.end_position[1] == x and self.end_position[0] == y):
                break

            for dir in range(0,4):
                x_s = x + diff[dir][0]
                y_s = y + diff[dir][1]

                if( x_s <= self.numberOfBox and x_s >= 0 and y_s <= self.numberOfBox and y_s >= 0 ):
                    if( self.start_value >= self.path_map[x_s][y_s] and self.path_map[x_s][y_s] !=123456 ):
                        x_p = x_s
                        y_p = y_s
                            
                        self.start_value = self.path_map[x_s][y_s]

            x = x_p
            y = y_p
        
        self.path = [x_path, y_path]    


    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle        
                         
    def calculate_angle(self, path):
        angle_tab = []

        for i in range(len(path)-1):
            angle = np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])
            if angle >= 3.1 and  path[i][1] < 0:
                angle = np.pi
            elif angle >= 3.1 and path[i][1] > 0:
                angle = -np.pi
            angle_tab.append(angle)
        
        return angle_tab

     