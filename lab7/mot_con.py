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
    global end_of_robot_position
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

        print("Robot position ", [position[0], position[1]])
        
        pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
        pixellate_map_obj.calculate_dists()
        pixellate_map_obj.path_finding()
        pixellate_map_obj.show_data(obstacle_map_msg_data)

        list_of_path= [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)] 
        path_map_msg_data.publish_path(list_of_path)

        list_of_path_from_thread = pixellate_map_obj.path
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
    global list_of_angle
    global end_angle_of_robot_position
    global end_of_robot_position

    end_angle_of_robot_position = 0.0
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
            print(list_of_path_from_thread)
            list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]
            print("Robot path ", list_of_path, "list_of_path_from_thread", list_of_path_from_thread) 
            set_correct_angle_position([list_of_path[1][0], list_of_path[1][1]], odom_raw_data, velocity_publisher, rate)

        if k == 0:
            velocity_publisher.publish_velocity(0.0, 0.0)

        if k > 2:

            list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]  
            list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
            list_of_angle.append(end_angle_of_robot_position)
            
            print("Robot path box ", list_of_path)
            print("Robot path angle ", list_of_angle)
            # robot_drive_thread = threading.Thread(target=robot_drive, args=( pid_linear, odom_raw_data, velocity_publisher, rate, laser_raw_data ), daemon=True)
            # robot_drive_thread.start()
            flag =0
            goals = list_of_path[1]
            angle = list_of_angle[1] 
            arrive_distance = 0.09
            while True:  
                if check_front( laser_raw_data, 0.2):  
                    velocity_publisher.publish_velocity(0.0, 0.0)
                    print("Obstacle in front of the robot")  
                    break
                goal = goals[0:2]
                if flag == 1:
                    flag = 0
                    break

                i=1

                # if check_front( laser_raw_data, velocity_publisher, odom_raw_data, rate):  
                #     velocity_publisher.publish_velocity(0.0, 0.0)    

                current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]] 
                distance_to_goal = np.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
                linearController = PID(kp = 0.10, ki =0.015, kd=0.001, limit=2)
                angularController = PID(kp = 0.25, ki =0.020, kd=0.002, limit=1)
                d_x = goal[0] - current[0]
                d_y = goal[1] - current[1]


                desired_angle = np.arctan2(d_y, d_x)

                
                current_angle = odom_raw_data.pose[2] 

            
                error_angle = desired_angle - current_angle

                w = angularController.calc(error_angle, 0)
                v = linearController.calc(distance_to_goal, 0)

                print("seting  goal",i," | ", goal, " >>", current)
                print("velocity", float(v), " ", float(w))
                velocity_publisher.publish_velocity(v, w) 
                rate.sleep()


                if distance_to_goal <  arrive_distance: 
                    angularController = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)

                    angle_tolerance = 0.01
                    while True:
                        list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]  
                        list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
                        list_of_angle.append(end_angle_of_robot_position)
                        desired_angle = list_of_angle[0]
                        current_angle = odom_raw_data.pose[2]  
                        error_angle = desired_angle - current_angle
                        print(list_of_path[0:3], list_of_angle[0:3])
                        print("seting  angle",i," | ", desired_angle, " >>>>", current_angle)


                        error_angle = desired_angle - current_angle
                        if abs(error_angle) < angle_tolerance:
                            velocity_publisher.publish_velocity(0.0, 0.0)
                            rate.sleep() 
                            flag = 1
                            break

                        w = angularController.calc(error_angle, 0)
                        print("velocity ", w)  
                        velocity_publisher.publish_velocity(0.0, float(w)) 
                        rate.sleep() 

                rate.sleep()

                if len(list_of_path) == 1:
                    end_of_path_flag = 1
                    break

            if end_of_path_flag == 1:
                break                     

        k+=1
        
    

    end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data) 
    rclpy.shutdown() 


if __name__ == '__main__':
    main()