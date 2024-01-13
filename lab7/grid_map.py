import pylab as pl
from math import cos, sin

class grid_map:
    def __init__(self):
        self.size = 25
        self.resolution_before_reducing = 0.1
        self.number_of_box_before_reducing = int(self.size/self.resolution_before_reducing)
        self.map =  pl.ones((self.number_of_box_before_reducing, self.number_of_box_before_reducing))/2
        self.prob_map =  pl.ones((self.number_of_box_before_reducing, self.number_of_box_before_reducing))/2
        self.center_before_reducing = int(self.number_of_box_before_reducing/2)
        self.filter_size = 5
        self.obstacle_threshold = 0.33

        self.numberOfBox = int(self.number_of_box_before_reducing /self.filter_size)
        self.drive_map = -pl.ones((self.number_of_box_before_reducing , self.number_of_box_before_reducing))

        self.hit_threshold = 100
        self.miss_threshold = -20
        self.p_hit = 0.90
        self.p_miss = 0.40

        self.sensor_displacement = 0#.15
        self.pathX = []
        self.pathY = []



    def printMap(self, pose):
        boxCordPos = self.box_coord(pose[0], pose[1])
        self.pathX.append(boxCordPos[0])
        self.pathY.append(boxCordPos[1])    
        self.show = pl.imshow( self.map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show( block=False )
        pl.pause( 0.1 )

    def print_drive_map(self):
        self.drive_show = pl.imshow( self.drive_map, interpolation="nearest", cmap='Blues', origin='lower')
        pl.show( block=False )

    def update_drive_map(self):
        self.drive_show.set_data(self.drive_map)



    def update(self, pose):
        # boxCordPos = self.box_coord(pose[0], pose[1])
        # self.pathX.append(boxCordPos[0])
        # self.pathY.append(boxCordPos[1])
        self.show.set_data(self.map)
        # pl.plot(self.pathX, self.pathY,'r-',linewidth=1)
        pl.show( block=False )
        pl.pause( 0.1 )



    def iterateLidar(self, dataScan, robot_position):
        min_angle = -2.356100082397461
        max_angle = 2.0932999382019043
        angle_increment = 0.006144199054688215


        theta = pl.arange(min_angle, max_angle, angle_increment)

        sensor_position = sensor_shift( self.sensor_displacement, 0, robot_position)
        
        for k in range(len(dataScan)):
            if pl.isinf(dataScan[k]) or pl.isnan(dataScan[k]):
                continue  

            pt = pol2Car(dataScan[k], theta[k], sensor_position)
            self.ray_trace( pt, sensor_position)

            obstacle_x = int(pt[0]/self.resolution_before_reducing) + self.center_before_reducing
            obstacle_y = int(pt[1]/self.resolution_before_reducing) + self.center_before_reducing
            # if(abs(pt[0] - sensor_position[1]) > 0.1 and abs(pt[1] - sensor_position[1])  > 0.1):
            self.map[obstacle_y][obstacle_x] = self.hit( self.map[obstacle_y][obstacle_x] )
        

        self.prob_map = self.computeProbab(self.map)

        
    def ray_trace(self, end, start):
        A = end[1] - start[1]
        B = start[0] - end[0]
        C = end[0] * start[1] - start[0] * end[1]

        dist_den = pl.sqrt(A*A + B*B)

        start_box = self.box_coord( start[0], start[1])
        end_box = self.box_coord( end[0], end[1])

        threshold = pl.sqrt(2)/2 * self.resolution_before_reducing

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
                
                real_x = (x - self.center_before_reducing) * self.resolution_before_reducing
                real_y = (y - self.center_before_reducing) * self.resolution_before_reducing
                center_hit = (abs( A*real_x + B*real_y + C) / dist_den) < threshold

                if( x_r <= self.number_of_box_before_reducing and x_r >= 0 and y_r <= self.number_of_box_before_reducing and y_r >= 0 ):
                    real_x_r = (x_r - self.center_before_reducing) * self.resolution_before_reducing
                    real_y_r = (y_r - self.center_before_reducing) * self.resolution_before_reducing
                    r_hit = (abs( A*real_x_r + B*real_y_r + C) / dist_den) < threshold
                    if( r_hit and real_x_r - x_r > 1 and real_y_r - y_r > 1):
                        self.map[y_r][x_r] = self.miss( self.map[y_r][x_r] )

                if( x_l <= self.number_of_box_before_reducing and x_l >= 0 and y_l <= self.number_of_box_before_reducing and y_l >= 0 ):
                    real_x_l = (x_l - self.center_before_reducing) * self.resolution_before_reducing
                    real_y_l = (y_l - self.center_before_reducing) * self.resolution_before_reducing
                    l_hit = (abs( A*real_x_l + B*real_y_l + C) / dist_den) < threshold
                    if( l_hit and real_x_r - x_r > 1 and real_y_r - y_r > 1):
                        self.map[y_l][x_l] = self.miss( self.map[y_l][x_l] )

                if( center_hit ):
                    self.map[y][x] = self.miss( self.map[y][x] )
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

                real_x = (x - self.center_before_reducing) * self.resolution_before_reducing
                real_y = (y - self.center_before_reducing) * self.resolution_before_reducing
                center_hit = (abs( A*real_x + B*real_y + C) / dist_den) < threshold

                if( x_r <= self.number_of_box_before_reducing and x_r >= 0 and y_r <= self.number_of_box_before_reducing and y_r >= 0 ):
                    real_x_r = (x_r - self.center_before_reducing) * self.resolution_before_reducing
                    real_y_r = (y_r - self.center_before_reducing) * self.resolution_before_reducing
                    r_hit = (abs( A*real_x_r + B*real_y_r + C) / dist_den) < threshold
                    if( r_hit and real_x_r - x_r > 1 and real_y_r - y_r > 1):
                        self.map[y_r][x_r] = self.miss( self.map[y_r][x_r] )

                if( x_l <= self.number_of_box_before_reducing and x_l >= 0 and y_l <= self.number_of_box_before_reducing and y_l >= 0 ):
                    real_x_l = (x_l - self.center_before_reducing) * self.resolution_before_reducing
                    real_y_l = (y_l - self.center_before_reducing) * self.resolution_before_reducing
                    l_hit = (abs( A*real_x_l + B*real_y_l + C) / dist_den) < threshold
                    if( l_hit and real_x_r - x_r > 1 and real_y_r - y_r > 1):
                        self.map[y_l][x_l] = self.miss( self.map[y_l][x_l] )

                if( center_hit ):
                    self.map[y][x] = self.miss( self.map[y][x] )
                elif( r_hit ):
                    x = x_r
                elif( l_hit ):
                    x = x_l



    def box_coord(self, x, y ):
        box_x = int( x / self.resolution_before_reducing) + self.center_before_reducing
        box_y = int( y / self.resolution_before_reducing) + self.center_before_reducing
        return pl.array([box_x, box_y])



    def map_coord(self, x, y ):
        real_x = (x - self.center_before_reducing) * self.resolution_before_reducing
        real_y = (y - self.center_before_reducing) * self.resolution_before_reducing
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



