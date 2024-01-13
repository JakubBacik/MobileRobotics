import rclpy
import pylab as pl
import threading
import numpy as np

from pixellate_map import pixellate_map
from grid_map import grid_map
from pid import PID_linear, PID_angular, PID
from publisher_subscriber_ros import odometry_node, velocity_node, laser_node
from additional_function import map_coord, box_coord
from matplotlib import pyplot as plt

global end_of_robot_position
global angle123
global list_of_path
global list_of_angle

def read_from_input():
    global end_of_robot_position
    global angle123
    print("To change the path, enter the new path in the form: x, y")
    while True:    
        number = input(".")
        number = number.split(' ')
        end_of_robot_position = [int(number[0]), int(number[1])]
        angle123 = float(number[2])

def init_node():
    laser_raw_data = laser_node()
    velocity_publisher = velocity_node()
    odom_raw_data = odometry_node()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser_raw_data)
    executor.add_node(odom_raw_data)
    executor.add_node(velocity_publisher)

    return laser_raw_data, velocity_publisher, odom_raw_data, executor

def check_front( laser_raw_data, threshold = 0.5):
    dataScan = laser_raw_data.lidar_buf.tolist()
    dataScan = dataScan[280:360]

    if min(dataScan) < threshold :
        return True
    else:
        return False


def end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor):
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    odom_raw_data.destroy_node()
    laser_raw_data.destroy_node()

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

        # Wrap the angle error to the range [-pi, pi]
        if abs(desired_angle - current_angle) > np.pi:
            if desired_angle > current_angle:
                desired_angle -= 2 * np.pi
            else:
                current_angle -= 2 * np.pi

        # If the error is within the tolerance, break the loop
        if abs(error_angle) < angle_tolerance:
            velocity_publisher.publish_velocity(0.0, 0.0)
            rate.sleep() 
            break

        # Calculate angular velocity using the angular controller
        w = angularController.calc(error_angle, 0)

        velocity_publisher.publish_velocity(0.0, w)
        rate.sleep()


def robot_map():
    



def main(args=None):
    global list_of_path
    global list_of_angle
    global angle123
    angle123 = 0.0
    rclpy.init()

    laser_raw_data, velocity_publisher, odom_raw_data, executor = init_node() 
    
    grid_map_obj = grid_map()
    pixellate_map_obj = pixellate_map()


    read_from_input_thread = threading.Thread(target=read_from_input, daemon=True)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = laser_raw_data.create_rate(8)
    
    while laser_raw_data.lidar_buf == 0:
        rate.sleep()
    
    global end_of_robot_position
    end_of_robot_position=[40, 25]
    pl.figure(figsize=(20, 10))
    pl.subplot(1, 3, 1)
    position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]

    pixellate_map_obj.set_end_position(end_of_robot_position)
    current_pos = box_coord(position[0], position[1], grid_map_obj)
    # print("Robot position ", [position[0], position[1]])
    pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
    
    grid_map_obj.iterateLidar( laser_raw_data.lidar_buf.tolist(), position)
    grid_map_obj.printMap(position)

    pl.subplot(1, 3, 2)
    pixellate_map_obj.prob_map = grid_map_obj.prob_map
    pixellate_map_obj.pixellate_map()
    pixellate_map_obj.print_drive_map()

    ax = pl.subplot(1, 3, 3)
    ax.set_xlim(0, pixellate_map_obj.numberOfBox-1)
    ax.set_ylim(0, pixellate_map_obj.numberOfBox-1)
    ax.grid()
    pixellate_map_obj.calculate_dists()
    pixellate_map_obj.print_path_map()
    

    read_from_input_thread.start()
    i = 1

    pid_linear = PID_linear(Kp=15.0, Ki=0.5, Kd=0.05, dt=0.1, arrive_distance=0.1, angle_distance=0.1, desiredV=0.1)
    pid_linear.current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]

    path_123 = []
    
    k = 0

    try:
        while rclpy.ok(): 
            print("============================== ", k)
            position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]        
            grid_map_obj.iterateLidar( laser_raw_data.lidar_buf.tolist(), position)
            grid_map_obj.update(position)
            pixellate_map_obj.prob_map = grid_map_obj.prob_map
            pixellate_map_obj.pixellate_map()
            pixellate_map_obj.update_drive_map()

            position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]
            pixellate_map_obj.clear_drive_map()
            pixellate_map_obj.set_end_position(end_of_robot_position)
            current_pos = box_coord(position[0], position[1], grid_map_obj)
            print("Robot position ", [position[0], position[1]])
            pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
            print("Robot BOX  position ", [current_pos[0], current_pos[1]])
            

            pixellate_map_obj.calculate_dists()
            pixellate_map_obj.update_path_map()
            
            
            ax.cla()
            ax.set_xlim(0, pixellate_map_obj.numberOfBox-1)
            ax.set_ylim(0, pixellate_map_obj.numberOfBox-1)
            pixellate_map_obj.print_path_map()

            pixellate_map_obj.path_finding()
            pixellate_map_obj.draw_path()  
            velocity_publisher.publish_velocity(0.0, 0.0)
            print("Robot path  ",pixellate_map_obj.path)
            
            if k == 2:
                list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*pixellate_map_obj.path)]  
                set_correct_angle_position([list_of_path[1][0], list_of_path[1][1]], odom_raw_data, velocity_publisher, rate)

            if k == 0:
                velocity_publisher.publish_velocity(0.0, 0.0)

            if k > 2:

                list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*pixellate_map_obj.path)]  
                list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
                list_of_angle.append(angle123)
                
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

                    # Angle from robot to goal
                    desired_angle = np.arctan2(d_y, d_x)

                # ...

                # Calculate the current angle error
                    
                    current_angle = odom_raw_data.pose[2] 

                    
                    # Handle the discontinuity at -π and π
                    if abs(desired_angle - current_angle) > np.pi:
                        if desired_angle > current_angle:
                            desired_angle -= np.pi
                        else:
                            current_angle -= np.pi

                    # Calculate the difference between the desired and current angles
                    error_angle = desired_angle - current_angle

                    w = angularController.calc(error_angle, 0)
                    v = linearController.calc(distance_to_goal, 0)

                    print("seting  goal",i," | ", goal, " >>", current)
                    velocity_publisher.publish_velocity(v, w) 
                    rate.sleep()


                    if distance_to_goal <  arrive_distance: # or abs(odom_raw_data.pose[2] - list_of_angle[i]) > 2.14:

                        angularController = PID(kp = 0.09, ki =0.015, kd=0.01, limit=0.25)
                        desired_angle = angle  # Set your desired angle here
                        angle_tolerance = 0.01
                        while True:
                            # Calculate the current angle error
                            current_angle = odom_raw_data.pose[2]  # Assuming this is the current angle
                            error_angle = desired_angle - current_angle
                            print("seting  angle",i," | ", desired_angle, " >>>>", current_angle)


                            # Wrap the angle error to the range [-pi, pi]
                            # error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi

                            # Handle the discontinuity at -π and π
                            if abs(desired_angle - current_angle) > np.pi:
                                if desired_angle > current_angle:
                                    desired_angle -= 2 * np.pi
                                else:
                                    current_angle -= 2 * np.pi

                            # Calculate the error
                            error_angle = desired_angle - current_angle
                            # If the error is within the tolerance, break the loop
                            if abs(error_angle) < angle_tolerance:
                                velocity_publisher.publish_velocity(0.0, 0.0)
                                rate.sleep() 
                                i = i+1
                                flag = 1
                                break

                            # Calculate angular velocity using the angular controller
                            # w = angularController.calc(desired_angle, current_angle)
                            w = angularController.calc(error_angle, 0)
                            velocity_publisher.publish_velocity(0.0, w) 
                            rate.sleep() 

                    rate.sleep()
                    if i == len(list_of_path):
                        break


            k+=1
        
     
    except KeyboardInterrupt:
        print('nie')
        pass
    
    # robot_drive_thread.join()
    end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor)    
    rclpy.shutdown()
    
    read_from_input_thread.join()
    executor_thread.join()




if __name__ == '__main__':
    main()