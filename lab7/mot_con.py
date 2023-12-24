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
from robot_controller import VelocityPublisher
from robot_controller import Controller
from robot_controller import Odometry_DataSubscriber

global path123


def map_coord(x, y, Map ):
    real_x = (x*Map.filter_size - Map.center) * Map.resolution
    real_y = (y*Map.filter_size - Map.center) * Map.resolution
    return [real_x, real_y, 0]
 
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
    path123=[70, 60]
    rclpy.init()
    data_sub = DataSubscriber()
    velocity_publisher = VelocityPublisher()
    Odometry = Odometry_DataSubscriber()

    Map = path_map()
    Drive_map = drive_map()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(data_sub)
    executor.add_node(Odometry)
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
    
    destination = [3,2]
    destination_of = []
    read_from_input_thread.start()
    try:
        i = 0
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


            if(len(Drive_map.path[0])>2 ):
                print("halo")
                if i==0:
                    destination_of = map_coord(Drive_map.path[1][1], Drive_map.path[0][1], Map)
                controller = Controller(Odometry.pose, destination_of)
                with open('somefile.txt', 'a') as the_file:
                    the_file.write(str(Odometry.pose) + " | " + str(destination_of) + "\n" )
                if( not controller.isArrived()):
                    i=1
                    destination_of = map_coord(Drive_map.path[1][1], Drive_map.path[0][1], Map)
                    print(str(Odometry.pose) + " | " + str(destination_of) + "\n" )
                    print("not yet")
                    v, w = controller.iteratePID()
                    velocity_publisher.publish_velocity(v, w)
                else:
                    print(str(Odometry.pose) + " | " + str(destination_of) + "\n" )
                    print("end")
                    velocity_publisher.publish_velocity(0.0, 0.0)


            rate.sleep()
        pl.show()
    except KeyboardInterrupt:
        pass
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    rclpy.shutdown()
    read_from_input_thread.join()
    executor_thread.join()




if __name__ == '__main__':
    main()
