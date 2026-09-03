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
        def get_min_dist(ranges_slice):
            valid = [r for r in ranges_slice if 0.1 < r < 10.0]
            return min(valid) if valid else 10.0

        # Hokuyo Lidar: index 180 este in fata (0 grade), index 0 este in spate (-180 grade)
        front_dist = get_min_dist(msg.ranges[150:210])  # Fata (180 +/- 30)
        left_dist = get_min_dist(msg.ranges[210:270])   # Stanga (180 -> 270)
        right_dist = get_min_dist(msg.ranges[90:150])   # Dreapta (90 -> 150)

        twist = Twist()
        
        if front_dist < 0.5:
            twist.linear.x = 0.0
            
            if left_dist > right_dist:
                twist.angular.z = 0.5
                self.get_logger().info(f'Ups! Incerc sa merg spre stanga....')
            else:
                twist.angular.z = -0.5
                self.get_logger().info(f'Ups! Incerc sa merg spre dreapta....')
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
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
            