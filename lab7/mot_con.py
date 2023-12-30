import rclpy
import pylab as pl
import threading
import numpy as np

from pixellate_map import pixellate_map
from grid_map import grid_map
from pid import PID_linear, PID_angular
from publisher_subscriber_ros import odometry_node, velocity_node, laser_node
from additional_function import map_coord, box_coord
from matplotlib import pyplot as plt

global end_of_robot_position
global list_of_path
global list_of_angle

def read_from_input():
    global end_of_robot_position
    print("To change the path, enter the new path in the form: x, y")
    while True:    
        number = input(".")
        number = number.split(' ')
        end_of_robot_position = [int(number[0]), int(number[1])]

def init_node():
    laser_raw_data = laser_node()
    velocity_publisher = velocity_node()
    odom_raw_data = odometry_node()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser_raw_data)
    executor.add_node(odom_raw_data)
    executor.add_node(velocity_publisher)

    return laser_raw_data, velocity_publisher, odom_raw_data, executor

def check_front( laser_raw_data, velocity_publisher, odom_raw_data, rate):
    if laser_raw_data.lidar_buf[359] < 0.2:
        return True
    else:
        return False


def end_node(laser_raw_data, velocity_publisher, odom_raw_data, executor):
    velocity_publisher.publish_velocity(0.0, 0.0)
    velocity_publisher.destroy_node()
    odom_raw_data.destroy_node()
    laser_raw_data.destroy_node()

def set_correct_angle_position(pid_linear, odom_raw_data, velocity_publisher, rate):
    angularController = PID_angular(Kp = 8.0, Ki =0.03, Kd=0.0, dt=0.1,  desired_w=2.0)
    desired_angle = np.arctan2(pid_linear.current[1] - odom_raw_data.pose[1],  pid_linear.current[0]-odom_raw_data.pose[0]) 
    angle_tolerance = 0.1

    while True:
        # Calculate the current angle error
        current_angle = odom_raw_data.pose[2] 
        print("set angle ", current_angle, " | " ,desired_angle)

        # print(desired_angle, " ", current_angle)
        error_angle = desired_angle - current_angle

        # Wrap the angle error to the range [-pi, pi]
        error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi
        # If the error is within the tolerance, break the loop
        if abs(error_angle) < angle_tolerance:
            velocity_publisher.publish_velocity(0.0, 0.0)
            #rate.sleep() 
            break

        # Calculate angular velocity using the angular controller
        w = angularController.control(error_angle)

        velocity_publisher.publish_velocity(0.0, w)
        #rate.sleep()

def robot_drive( pid_linear, odom_raw_data, velocity_publisher, rate, laser_raw_data):
    global list_of_path
    global list_of_angle

    i = 0
    while True:  
        print(i) 
        # if abs(odom_raw_data.pose[0] - list_of_path[1][0]) < 0.2 and abs(odom_raw_data.pose[1] - list_of_path[1][1]) < 0.2:
        #     i=2
        # else:
        #     i=1

        if check_front( laser_raw_data, velocity_publisher, odom_raw_data, rate):  
            velocity_publisher.publish_velocity(0.0, 0.0)    

        pid_linear.current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]] 
        pid_linear.goal = list_of_path[i] 
        pid_linear.goal_angle = list_of_angle[i]
        print("set linear ", pid_linear.current , " | " , pid_linear.goal)

        v, w = pid_linear.iteratePID()

        velocity_publisher.publish_velocity(v, w) 
        

        distance_to_goal = np.sqrt((pid_linear.goal[0] - pid_linear.current[0])**2 + (pid_linear.goal[1] - pid_linear.current[1])**2)

        if distance_to_goal < pid_linear.arrive_distance or abs(odom_raw_data.pose[2] - list_of_angle[i]) > 2.14:
            angularController = PID_angular(Kp = 10.0, Ki =0.5, Kd=0.01, dt=0.1,  desired_w=20.0 )
            desired_angle = list_of_angle[i]  
            angle_tolerance = 0.2

            while True:
                current_angle = odom_raw_data.pose[2]  
                print("set angle123 ",current_angle , " | " ,desired_angle)
                error_angle = desired_angle - current_angle
                error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi


                if abs(error_angle) < angle_tolerance:
                    velocity_publisher.publish_velocity(0.0, 0.0)
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    rate.sleep() 
                    i=i+1
                    break

                # Calculate angular velocity using the angular controller
                w = angularController.control(error_angle)

                velocity_publisher.publish_velocity(0.0, w) 
                rate.sleep() 

        rate.sleep()
        if i == len(list_of_path):
            break


def main(args=None):
    global list_of_path
    global list_of_angle
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
    end_of_robot_position=[30, 30]
    pl.figure(figsize=(20, 10))
    pl.subplot(1, 3, 1)
    position = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]
    grid_map_obj.iterateLidar( laser_raw_data.lidar_buf.tolist(), position)
    grid_map_obj.printMap(position)

    pl.subplot(1, 3, 2)
    pixellate_map_obj.prob_map = grid_map_obj.prob_map
    pixellate_map_obj.pixellate_map()
    pixellate_map_obj.print_drive_map()

    ax = pl.subplot(1, 3, 3)
    ax.set_xlim(0, pixellate_map_obj.numberOfBox-1)
    ax.set_ylim(0, pixellate_map_obj.numberOfBox-1)
    pixellate_map_obj.calculate_dists()
    pixellate_map_obj.print_path_map()
    

    read_from_input_thread.start()
    i = 1

    pid_linear = PID_linear(Kp=15.0, Ki=0.5, Kd=0.05, dt=0.1, arrive_distance=0.1, angle_distance=0.1, desiredV=0.1)
    pid_linear.current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]]

    velocity_publisher.publish_velocity(0.1, 0.3) 
    # set_correct_angle_position(pid_linear, odom_raw_data, velocity_publisher, rate)
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
            print("Robot BOX position ", [position[0], position[1]])
            pixellate_map_obj.set_start_position([current_pos[0], current_pos[1]])
            print("Robot position ", [current_pos[0], current_pos[1]])
            

            pixellate_map_obj.calculate_dists()
            pixellate_map_obj.update_path_map()
            
            
            ax.cla()
            ax.set_xlim(0, pixellate_map_obj.numberOfBox-1)
            ax.set_ylim(0, pixellate_map_obj.numberOfBox-1)
            pixellate_map_obj.print_path_map()

            pixellate_map_obj.path_finding()
            pixellate_map_obj.draw_path()
            print("Robot path  ",pixellate_map_obj.path)
            
            

            if k > 2:
                list_of_path = [map_coord(y, x, grid_map_obj) for x, y in zip(*pixellate_map_obj.path)]  
                list_of_angle = pixellate_map_obj.calculate_angle(list_of_path)
                list_of_angle.append(list_of_angle[-1])
                
                print("Robot path box ", list_of_path)
                # robot_drive_thread = threading.Thread(target=robot_drive, args=( pid_linear, odom_raw_data, velocity_publisher, rate, laser_raw_data ), daemon=True)
                # robot_drive_thread.start()
                flag =0
                while True:  
                    if flag == 1:
                        flag = 0
                        break

                    # if abs(odom_raw_data.pose[0] - list_of_path[1][0]) < 0.2 and abs(odom_raw_data.pose[1] - list_of_path[1][1]) < 0.2:
                    #     i=2
                    # else:
                    #     i=1
                    
                    # if(current_pos[0] == pixellate_map_obj.path[1][0] and current_pos[1] == pixellate_map_obj.path[0][0]):
                    #     print("zmiana")
                    #     i=2
                    # else:
                    #     i=1
                    i=1

                    if check_front( laser_raw_data, velocity_publisher, odom_raw_data, rate):  
                        velocity_publisher.publish_velocity(0.0, 0.0)    

                    pid_linear.current = [odom_raw_data.pose[0], odom_raw_data.pose[1], odom_raw_data.pose[2]] 
                    pid_linear.goal = list_of_path[i] 
                    pid_linear.goal_angle = list_of_angle[i]
                    print("set linear ", pid_linear.current , " | " , pid_linear.goal)

                    v, w = pid_linear.iteratePID()

                    velocity_publisher.publish_velocity(v, w) 
                    

                    distance_to_goal = np.sqrt((pid_linear.goal[0] - pid_linear.current[0])**2 + (pid_linear.goal[1] - pid_linear.current[1])**2)

                    if distance_to_goal < pid_linear.arrive_distance:# or abs(odom_raw_data.pose[2] - list_of_angle[i]) > 2.14:
                        # angularController = PID_angular(Kp = 10.0, Ki =0.5, Kd=0.2, dt=0.1,  desired_w=30.0 )
                        # angularController = PID_angular(Kp = 15.0, Ki =0.8, Kd=0.3, dt=0.1,  desired_w=30.0 )
                        angularController = PID_angular(Kp = 15.0, Ki =0.8, Kd=0.3, dt=0.1,  desired_w=30.0 )
                        desired_angle = list_of_angle[i]  
                        angle_tolerance = 0.01

                        while True:
                            current_angle = odom_raw_data.pose[2]  
                            print("set angle123 ",current_angle , " | " ,desired_angle)
                            error_angle = desired_angle - current_angle
                            error_angle = (error_angle + np.pi) % (2 * np.pi) - np.pi


                            if abs(error_angle) < angle_tolerance:
                                velocity_publisher.publish_velocity(0.0, 0.0)
                                rate.sleep() 
                                rate.sleep() 
                                rate.sleep() 
                                flag = 1
                                print("flag", flag)
                                break

                            # Calculate angular velocity using the angular controller
                            w = angularController.control(error_angle)

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