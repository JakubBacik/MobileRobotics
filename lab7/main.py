import rclpy
import threading
from publisher_subscriber_ros import laser_node


def run_subscriber_and_publisher_thread():
    laser = laser_node()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)

    return executor_thread

def main(args=None):
    rclpy.init(args=args)
    laser = laser_node()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    while rclpy.ok():
        print(laser.lidar_buf)

    laser.destroy_node()
    executor_thread.join()
    rclpy.shutdown()