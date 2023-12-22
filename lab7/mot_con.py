import pylab as pl
import rclpy
import time
import numpy as np
from rclpy.node import Node, Executor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
import threading
from drive_map import drive_map
from path_plan import path_map
from data_subscriber import DataSubscriber
from path_plan import get_raw_data_bulk

    


# if __name__ == "__main__":
#     path = path_from_data()
#     print(path[0])
#     print(path[1])

#     pl.plot(path[1], path[0],'r-',linewidth=1)
#     pl.show( )
#     pl.pause( 0.1 )

#     path_len = len(path[0])
#     for i in range(0, path_len-1):
#         if( path[0][i] == path[0][i]):
#             if( path[1][i+1] < path[1][i]):
#                 set_dir()
global path123

        
def box_coord(x, y, Map ):
    box_x = int( x / Map.resolution) + Map.center
    box_y = int( y / Map.resolution) + Map.center
    return [int(box_x/Map.filter_size), int(box_y/Map.filter_size)]

def read_from_input():
    global path123
    print("To change the path, enter the new path in the form: x, y")
    while True:    
        number = input(".")
        number = number.split(' ')
        path123 = [int(number[0]), int(number[1])]
    

def main(args=None):
    global path123
    path123=[2, 2]
    rclpy.init()
    data_sub = DataSubscriber()

    Map = path_map()
    Drive_map = drive_map()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(data_sub)
    read_from_input_thread = threading.Thread(target=read_from_input, daemon=True)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = data_sub.create_rate(10)
    
    while data_sub.lidar_buf == 0:
        rate.sleep()

    
    pl.subplot(1, 3, 1)
    position = [data_sub.pose[0], data_sub.pose[1], data_sub.pose[2]]
    Map.iterateLidar( data_sub.lidar_buf[103:615].tolist(), position)
    Map.printMap(position)

    pl.subplot(1, 3, 2)
    Drive_map.prob_map = Map.prob_map
    Drive_map.pixellate_map()
    Drive_map.print_drive_map()

    ax = pl.subplot(1, 3, 3)
    ax.set_xlim(0, Drive_map.numberOfBox-1)
    ax.set_ylim(0, Drive_map.numberOfBox-1)
    Drive_map.calculate_dists()
    Drive_map.print_path_map()
    
    destination = [23,15]
    read_from_input_thread.start()
    try:
        while rclpy.ok(): 
            position = [data_sub.pose[0], data_sub.pose[1], data_sub.pose[2]]        
            Map.iterateLidar( data_sub.lidar_buf[103:615].tolist(), position)
            Map.update(position)
            Drive_map.prob_map = Map.prob_map
            Drive_map.pixellate_map()
            Drive_map.update_drive_map()

            position = [int(data_sub.pose[0]), int(data_sub.pose[1]), int(data_sub.pose[2])]
            Drive_map.clear_drive_map()
            Drive_map.set_end_position(path123)
            current_pos = box_coord(position[0], position[1], Map)
            Drive_map.set_start_position([current_pos[0], current_pos[1]])

            Drive_map.calculate_dists()
            Drive_map.update_path_map()
            
            ax.cla()
            ax.set_xlim(0, Drive_map.numberOfBox-1)
            ax.set_ylim(0, Drive_map.numberOfBox-1)
            Drive_map.print_path_map()

            Drive_map.path_finding()
            Drive_map.draw_path()


 
            rate.sleep()
        pl.show()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    read_from_input_thread.join()
    executor_thread.join()




if __name__ == '__main__':
    main()
