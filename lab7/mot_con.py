import rclpy
import pylab as pl
import threading
import sys
import numpy as np
import time
from pixellate_map import pixellate_map
from grid_map import grid_map
from pid import  PID
from publisher_subscriber_ros import odometry_node, velocity_node, laser_node, map_msg, obstacle_map_msg, path_map_msg
from additional_function import map_coord, box_coord
from matplotlib import pyplot as plt

global end_of_robot_position
global end_angle_of_robot_position
global list_of_path
global list_of_angle

def read_from_input():
    global end_of_robot_position
    global end_angle_of_robot_position
    print("To change the path, enter the new path in the form: x, y")
    while True:    
        number = input(".")
        number = number.split(' ')
        end_of_robot_position = [int(number[0]), int(number[1])]
        end_angle_of_robot_position = float(number[2])

def init_node():
    laser_raw_data = laser_node()
    velocity_publisher = velocity_node()
    odom_raw_data = odometry_node()
    map_msg_data = map_msg()
    obstacle_map_msg_data = obstacle_map_msg()
    path_map_msg_data = path_map_msg()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser_raw_data)
    executor.add_node(odom_raw_data)
    executor.add_node(velocity_publisher)
    executor.add_node(map_msg_data)
    executor.add_node(obstacle_map_msg_data)
    executor.add_node(path_map_msg_data)

    return laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data

def end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data):
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    odom_raw_data.destroy_node()
    laser_raw_data.destroy_node()
    map_msg_data.destroy_node()
    obstacle_map_msg_data.destroy_node()
    path_map_msg_data.destroy_node()

def check_front( laser_raw_data, threshold = 0.5):
    dataScan = laser_raw_data.lidar_buf.tolist()
    dataScan = dataScan[280:360]

    if min(dataScan) < threshold :
        return True
    else:
        return False



def robot_map(odom_raw_data, grid_map_obj, pixellate_map_obj, laser_raw_data, map_msg_data, obstacle_map_msg_data, path_map_msg_data):
    global list_of_path_from_thread
    global list_of_angle_from_thread
    global end_angle_of_robot_position
    global end_of_robot_position
    end_angle_of_robot_position = 0.0   
    end_of_robot_position=[40, 25]
    while True:
        position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]        
        grid_map_obj.iterateLidar( laser_raw_data.lidar_buf.tolist(), position)

        grid_map_obj.show_data(map_msg_data)

        pixellate_map_obj.prob_map = grid_map_obj.prob_map
        pixellate_map_obj.pixellate_map()

        position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]
        pixellate_map_obj.clear_drive_map()
        pixellate_map_obj.set_end_position(end_of_robot_position)
        current_pos = box_coord(position[0], position[1], grid_map_obj)
        
        pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
        pixellate_map_obj.calculate_dists()
        pixellate_map_obj.path_finding()
        pixellate_map_obj.show_data(obstacle_map_msg_data)      

        list_of_path_12 = pixellate_map_obj.path
        list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_12)] 

        path_map_msg_data.publish_path(list_of_path)

        list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
        list_of_angle.append(end_angle_of_robot_position)

        list_of_path_from_thread = list_of_path
        list_of_angle_from_thread = list_of_angle


        time.sleep(0.5)


def set_correct_angle_position(current, odom_raw_data, velocity_publisher, rate):
    angularController = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)
    desired_angle = np.arctan2(current[1] - odom_raw_data.pose[1],  current[0]-odom_raw_data.pose[0]) 
    angle_tolerance = 0.1

    while True:
        # Calculate the current angle error
        current_angle = odom_raw_data.pose[2] 
        print("set angle ", current_angle, " | " ,desired_angle)

        # print(desired_angle, " ", current_angle)
        error_angle = desired_angle - current_angle

        # If the error is within the tolerance, break the loop
        if abs(error_angle) < angle_tolerance:
            velocity_publisher.publish_velocity(0.0, 0.0)
            rate.sleep() 
            break

        # Calculate angular velocity using the angular controller
        w = angularController.calc(error_angle, 0)

        velocity_publisher.publish_velocity(0.0, w)
        rate.sleep()

def main(args=None):
    global list_of_path_from_thread
    list_of_path_from_thread= []
    global list_of_angle_from_thread
    list_of_angle_from_thread= []
    global list_of_angle
    global end_of_robot_position

    end_of_path_flag = 0

    rclpy.init()
    laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data = init_node() 
    
    grid_map_obj = grid_map()
    pixellate_map_obj = pixellate_map()

    read_from_input_thread = threading.Thread(target=read_from_input, daemon=True)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    path_map_creator = threading.Thread(target=robot_map, args=(odom_raw_data, grid_map_obj, pixellate_map_obj, laser_raw_data, map_msg_data, obstacle_map_msg_data, path_map_msg_data), daemon=True)
    
    executor_thread.start()

    rate = laser_raw_data.create_rate(8)
    while laser_raw_data.lidar_buf == 0:
        rate.sleep()

    read_from_input_thread.start()
    path_map_creator.start()

    k = 0
    while rclpy.ok(): 
        while len(list_of_path_from_thread) == 0:
            rate.sleep()
            
        if k == 2:
            set_correct_angle_position([list_of_path_from_thread[1][0], list_of_path_from_thread[1][1]], odom_raw_data, velocity_publisher, rate)

        if k == 0:
            velocity_publisher.publish_velocity(0.0, 0.0)

        if k > 2:

            flag =0
            next_position = list_of_path_from_thread[1]
            arrive_distance = 0.09
            angle_tolerance = 0.01
            linearController = PID(kp = 0.10, ki =0.015, kd=0.001, limit=2)
            angularController = PID(kp = 0.25, ki =0.020, kd=0.002, limit=1)

            while True:  
                if check_front( laser_raw_data, 0.2):  
                    velocity_publisher.publish_velocity(0.0, 0.0)
                    print("Obstacle in front of the robot")  
                    break

                if flag == 1:
                    flag = 0
                    break

                distance_to_goal = np.sqrt((next_position[0] - odom_raw_data.pose[0])**2 + (next_position[1] - odom_raw_data.pose[1])**2)
                
                error_angle = np.arctan2(next_position[1] - odom_raw_data.pose[1], next_position[0] - odom_raw_data.pose[0]) - odom_raw_data.pose[2] 

                w = angularController.calc(error_angle, 0)
                v = linearController.calc(distance_to_goal, 0)

                print("Desired position", next_position[0], " ", next_position[1], " current: ", round(odom_raw_data.pose[0], 3), " ", round(odom_raw_data.pose[1], 3))
                velocity_publisher.publish_velocity(v, w) 
                rate.sleep()


                if distance_to_goal <  arrive_distance: 
                    angularController = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)

                    while True:
                        
                        error_angle = list_of_angle_from_thread[0] - odom_raw_data.pose[2]
                        print("Desired angle: " , round(list_of_angle_from_thread[0], 3), " current: ", round(odom_raw_data.pose[2], 3))
                        if abs(error_angle) < angle_tolerance:
                            velocity_publisher.publish_velocity(0.0, 0.0)
                            rate.sleep() 
                            flag = 1
                            break

                        w = angularController.calc(error_angle, 0)
                        velocity_publisher.publish_velocity(0.0, float(w)) 
                        rate.sleep() 

                rate.sleep()
                # print(list_of_path_from_thread, len(list_of_path_from_thread))
                if len(list_of_path_from_thread) == 1:
                    end_of_path_flag = 1
                    break

            if end_of_path_flag == 1:
                break                     

        k+=1
        
    

    end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data) 
    rclpy.shutdown() 


if __name__ == '__main__':
    main()