import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def callback_scan(msg):
    ## extract ranges from message
    scan=list(msg.ranges)
    print("  Scan min: %f  front: %f" % ( min(scan),scan[362]))
    print ()


def main(args=None):
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: sys.argv[0] \n')
        sys.exit(1)
    rclpy.init()
    nr=sys.argv[1]

    node = Node('listener')

    # Subscribe topics and bind with callback functions
    node.create_subscription(LaserScan, f"/pioneer{nr}/scan", callback_scan, 10)

    # spin(node) simply keeps python from exiting until this node is stopped
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
