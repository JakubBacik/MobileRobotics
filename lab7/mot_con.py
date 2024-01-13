import rclpy
import pylab as pl
import threading
import numpy as np
import time
from pixellate_map import pixellate_map
from grid_map import grid_map
from pid import PID
from publisher_subscriber_ros import *
from additional_function import map_coord, box_coord
from matplotlib import pyplot as plt



global user_specified_position
global user_specified_angle
global list_of_path
global list_of_angle



def read_from_input():
    global user_specified_position
    global user_specified_angle
    print("To change the path, enter the new path in the form: x, y")
    while True:    
        number = input(".")
        number = number.split(' ')
        user_specified_position = [int(number[0]), int(number[1])]
        user_specified_angle = float(number[2])



def init_nodes():
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



def check_front( laser_raw_data, threshold = 0.5):
    dataScan = laser_raw_data.lidar_buf.tolist()
    dataScan = dataScan[280:360]

    if min(dataScan) < threshold :
        return True
    else:
        return False



def end_nodes(laser_raw_data, velocity_publisher, odom_raw_data, map_msg_data, obstacle_map_msg_data):
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    odom_raw_data.destroy_node()
    laser_raw_data.destroy_node()
    map_msg_data.destroy_node()
    obstacle_map_msg_data.destroy_node()



def set_correct_angle_position(current, odom_raw_data, velocity_publisher):
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
            time.sleep(0.1) 
            break

        # Calculate angular velocity using the angular controller
        w = angularController.calc(error_angle, 0)

        velocity_publisher.publish_velocity(0.0, w)
        time.sleep(0.1)



def robot_map( pixellate_map_obj, odom_raw_data, laser_raw_data, velocity_publisher, map_msg_data, obstacle_map_msg_data, path_map_msg_data):
    global list_of_path_from_thread

    grid_map_obj = grid_map()

    while True:
        position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]  
        current_pos = box_coord(position[0], position[1], grid_map_obj)
        # print("Robot position ", [position[0], position[1]])
      
        grid_map_obj.iterateLidar( laser_raw_data.lidar_buf.tolist(), position)
        grid_map_obj.show_data(map_msg_data)

        pixellate_map_obj.prob_map = grid_map_obj.prob_map
        pixellate_map_obj.pixellate_map()
        pixellate_map_obj.clear_drive_map()
        pixellate_map_obj.set_end_position(user_specified_position)
        pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
        pixellate_map_obj.calculate_dists()
        pixellate_map_obj.path_finding()
        pixellate_map_obj.show_data(obstacle_map_msg_data)

        list_of_path123 = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)] 
        # print("Robot path  ",list_of_path123)
        path_map_msg_data.publish_path(list_of_path123)
        list_of_path_from_thread = pixellate_map_obj.path
        time.sleep(0.5)




def main(args=None):
    global list_of_path
    global list_of_path_from_thread
    global list_of_angle
    global user_specified_angle
    global user_specified_position
    
    angle_tolerance = 0.01
    distance_tolerance = 0.09
    user_specified_angle = 0.0
    list_of_path_from_thread = []
    user_specified_position = [40, 25]
    iteration = 0
    finished = 0

    rclpy.init()

    laser_raw_data, velocity_publisher, odom_raw_data, executor, map_msg_data, obstacle_map_msg_data, path_map_msg_data = init_nodes() 
    pixellate_map_obj = pixellate_map()

#    read_from_input_thread = threading.Thread(target=read_from_input, daemon=True)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    path_map_creator = threading.Thread(target=robot_map, args=(pixellate_map_obj, odom_raw_data, laser_raw_data, velocity_publisher, map_msg_data, obstacle_map_msg_data, path_map_msg_data), daemon=True)

    executor_thread.start()
    
    while laser_raw_data.lidar_buf == 0:
        time.sleep(0.1)
    
#    read_from_input_thread.start()
    path_map_creator.start()
    
    linearController = PID(kp = 0.10, ki =0.015, kd=0.001, limit=2)
    angularController = PID(kp = 0.25, ki =0.020, kd=0.002, limit=1)
    angle_controller = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)

    try:
        while rclpy.ok(): 
            # print("============================== ", iteration)
            if iteration == 0:
                velocity_publisher.publish_velocity(0.0, 0.0)   
                while len(list_of_path_from_thread) == 0:
                    time.sleep(0.1)
                
            if iteration == 1: # pretty sure 1 is right this time
                print(list_of_path_from_thread)
                list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]
                print("Robot path ", list_of_path, "list_of_path_from_thread", list_of_path_from_thread) 
                set_correct_angle_position([list_of_path[1][0], list_of_path[1][1]], odom_raw_data, velocity_publisher)

            if iteration > 1:

                list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]  
                list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
                list_of_angle.append(user_specified_angle)
                
#                print("Robot path box ", list_of_path)
#                print("Robot path angle ", list_of_angle)

                is_in_position = 0
                goals = list_of_path[1]
                angle = list_of_angle[1] 

                while True:  
                    if check_front( laser_raw_data, 0.2):  
                        velocity_publisher.publish_velocity(0.0, 0.0)
                        print("Obstacle in front of the robot")  
                        break
                    
                    if is_in_position == 1:
                        is_in_position = 0
                        break

                    goal = goals[0:2]

                    current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]] 
                    distance_to_goal = np.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
                    
                    d_x = goal[0] - current[0]
                    d_y = goal[1] - current[1]

                    # Angle from robot to goal
                    desired_angle = np.arctan2(d_y, d_x)
                    
                    current_angle = odom_raw_data.pose[2] 

                    # Calculate the difference between the desired and current angles
                    error_angle = desired_angle - current_angle

                    w = angularController.calc(error_angle, 0)
                    v = linearController.calc(distance_to_goal, 0)

                    print("seting  goal | ", goal, " >>", current)
                    print("velocity", float(v), " ", float(w))
                    velocity_publisher.publish_velocity(v, w) 
                    time.sleep(0.1)


                    if distance_to_goal < distance_tolerance: 
                        while True:
                            list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*list_of_path_from_thread)]  
                            list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
                            list_of_angle.append(user_specified_angle)
                            desired_angle = list_of_angle[0]
                            # Calculate the current angle error
                            current_angle = odom_raw_data.pose[2]  # Assuming this is the current angle
                            error_angle = desired_angle - current_angle
                            print(list_of_path[0:3], list_of_angle[0:3])
                            print("seting  angle | ", desired_angle, " >>>>", current_angle)

                            # Calculate the error
                            error_angle = desired_angle - current_angle
                            # If the error is within the tolerance, break the loop
                            if abs(error_angle) < angle_tolerance:
                                velocity_publisher.publish_velocity(0.0, 0.0)
                                time.sleep(0.1) 
                                is_in_position = 1
                                break

                            # Calculate angular velocity using the angular controller
                            w = angle_controller.calc(error_angle, 0)
                            print("velocity ", w)  
                            velocity_publisher.publish_velocity(0.0, float(w)) 
                            time.sleep(0.1) 

                    time.sleep(0.1)
                    if 1 == len(list_of_path):
                        print("jeden")
                        finished = 1
                        break

                if finished == 1:
                    print("dwea")
                    break
                        
            if finished == 1:
                print('trzty')
                break                      

            iteration += 1
        
     
    except KeyboardInterrupt: # excludes read_from_input as it uses the same console
        print('nie')
        pass
    


    end_nodes(laser_raw_data, velocity_publisher, odom_raw_data, map_msg_data, obstacle_map_msg_data, path_map_msg_data)    
    rclpy.shutdown()
    path_map_creator.join()
#    read_from_input_thread.join()
    executor_thread.join()




if __name__ == '__main__':
    main()
