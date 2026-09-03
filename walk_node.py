#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist # for liniar and angular speed
from rclpy.qos import qos_profile_sensor_data # so it doesn't use best effort automatically, instead it uses reliable, to avoid losing messages

class WalkNode(Node):
    def __init__(self):
        super().__init__('walk_node') # give a name to the ROS2 node
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data) # subscribe to LIDAR
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Cautare cale libera.....')

    def scan_callback(self, msg):
        front_ranges = msg.ranges[:45] + msg.ranges[-45:]

        valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0] # filter out invalid values

        twist = Twist()
        if valid_ranges and min(valid_ranges) < 0.5: # if there is somethinf cloder than 0.5m in front of pup
            twist.angular.z = -0.5 # turn right a bit
            self.get_logger().info('Ups, nu pot merge inainte, reconfigurare traseu....')
        else:
            twist.linear.x = 0.2 # move forward
            self.get_logger().info('Cale libera, inainte....')

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    walk_node = WalkNode()
    rclpy.spin(walk_node)
    walk_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            