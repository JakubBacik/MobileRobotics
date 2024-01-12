import pylab as pl
import numpy as np


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
    
    def set_end_position(self, end_position):
        self.end_position = end_position

    def print_drive_map(self):
        self.drive_show = pl.imshow( self.drive_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show(block=False)

    def update_drive_map(self):
        self.drive_show.set_data(self.drive_map)

    def print_path_map(self):
        self.path_show = pl.imshow( self.path_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show( block=False )
    
    def update_path_map(self):
        self.path_show.set_data(self.path_map)

    def draw_path(self):
        pl.plot(self.path[1], self.path[0],'r-',linewidth=1)
        pl.plot(self.start_position[0], self.start_position[1],'go',linewidth=1)
        pl.plot(self.end_position[0], self.end_position[1],'go',linewidth=1)
        pl.text(self.start_position[0]-0.5, self.start_position[1]+0.5, 'Start', fontsize=5)
        pl.text(self.end_position[0]-0.5, self.end_position[1]+0.5, 'End', fontsize=5)
        pl.show(block=False )


    # def pixellate_map(self):    
    #     for x in range(0, self.numberOfBoxSmall, self.filter_size):
    #         for y in range(0, self.numberOfBoxSmall, self.filter_size): 
    #             for x_s in range(x, x+self.filter_size):
    #                 for y_s in range( y, y+self.filter_size):
    #                     if self.prob_map[x_s][y_s] > self.obstacle_threshold:
    #                         self.drive_map[int(x/self.filter_size)][int(y/self.filter_size)] = 123456


    def pixellate_map(self):    
        for x in range(0, self.numberOfBoxSmall, self.filter_size):
            for y in range(0, self.numberOfBoxSmall, self.filter_size): 
                # Initialize a flag to check if there's an obstacle in the current block
                obstacle_in_block = False
                for x_s in range(x, min(x+self.filter_size, self.numberOfBoxSmall)):
                    for y_s in range(y, min(y+self.filter_size, self.numberOfBoxSmall)):
                        if self.prob_map[x_s][y_s] > self.obstacle_threshold:
                            obstacle_in_block = True
                            break
                    if obstacle_in_block:
                        break
                # If there's an obstacle in the current block, mark the corresponding cell in the drive_map as an obstacle
                if obstacle_in_block:
                    # Round the coordinates to the nearest multiple of self.filter_size before dividing
                    self.drive_map[round(x/self.filter_size)][round(y/self.filter_size)] = 123456

    def clear_drive_map(self):
        self.path_map = -pl.ones((self.numberOfBox, self.numberOfBox))
        self.path = []

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

     