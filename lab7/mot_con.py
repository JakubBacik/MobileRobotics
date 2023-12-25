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
from robot_controller import Odometry_DataSubscriber
from pid import PID
from robot_controller import RobotController
from matplotlib import pyplot as plt

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
    pathsize = 0
    global path123
    path123=[40, 40]
    rclpy.init()
    data_sub = DataSubscriber()
    velocity_publisher = VelocityPublisher()
    Odometry = Odometry_DataSubscriber()

    Map = path_map()
    Drive_map = drive_map()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(data_sub)
    executor.add_node(Odometry)
    executor.add_node(velocity_publisher)
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
    i = 0
    controller = RobotController(Kp=7.0, Ki=0.5, Kd=0.01, dt=0.1, arrive_distance=0.1, angle_distance=0.1, desiredV=0.1)
    controller.current = [Odometry.pose[0], Odometry.pose[1], Odometry.pose[2]]  # x, y, angle 
  

    try:
        j = 0
        i = 1
        old_pathsize = 0
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
            Drive_map.calculate_angle()
            Drive_map.draw_path()
            velocity_publisher.publish_velocity(0.0, 0.2) 

            # pathsize = len(Drive_map.path[0])
            # if  pathsize == old_pathsize:
            #     i+=1
            # else:
            #     i=1
            
            
            # print("halo")

            # if(j > 2):
            #     while True:                  
            #         controller.current = [Odometry.pose[0], Odometry.pose[1], Odometry.pose[2]] 
            #         controller.goal = map_coord(Drive_map.path[1][i], Drive_map.path[0][i], Map)

            #         v, w = controller.iteratePID()

            #         # Calculate the angle error
            #         error_x = controller.goal[0] - controller.current[0]
            #         error_y = controller.goal[1] - controller.current[1]
            #         # print(controller.goal[0], ' ', controller.goal[1])
            #         # print(Odometry.pose)
            #         error_angle = np.arctan2(error_y, error_x) - controller.current[2]

            #         # Wrap the angle error to the range [-pi, pi]
            #         error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi
            #         flag = 0
            #         # If the absolute value of the angle error is greater than pi/2, make a U-turn
            #         if abs(error_angle) > np.pi / 2.8:
            #             flag =1
            #             print("Making a U-turn")
            #             v= 0.0
            #             w = np.sign(w)#* controller.desiredV  # Use the maximum angular velocity for the U-turn
                        

            #         if flag == 0:
            #             velocity_publisher.publish_velocity(v, w)  
            #             rate.sleep()
            #         else:
            #             velocity_publisher.publish_velocity(v, w)
            #             time.sleep(1)
            #             flag=0
                    
            #         print(controller.goal[0], ' ', controller.goal[1], controller.goal[2])
            #         print(Odometry.pose) 
            #         distance_to_goal = np.sqrt((controller.goal[0] - controller.current[0])**2 + (controller.goal[1] - controller.current[1])**2)
            #         if distance_to_goal < controller.arrive_distance:
            #             velocity_publisher.publish_velocity(0.0, 0.0) 
            #             rate.sleep()
            #             old_pathsize = pathsize
            #             print(f"===================================================== {i}")
            #             break
                        
            
            #         if i == len(Drive_map.path)-1:
            #             print("end of path")
            #             break
                    
            # j = j+1


            # rate.sleep()
        
    except KeyboardInterrupt:
        pass
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    rclpy.shutdown()
    read_from_input_thread.join()
    executor_thread.join()




if __name__ == '__main__':
    main()
