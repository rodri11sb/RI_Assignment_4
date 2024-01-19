#!/usr/bin/env python3

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from flatland_msgs.msg import Collisions

class LeftRightRobot(Node):
    def __init__(self):
        super().__init__("LeftRightRobot")

        self.linear_speed = 0.5
        self.angular_speed = 0.0  # Initial angular speed

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)
        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)

    def change_robot_speeds(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.pub.publish(twist_msg)

    def processLiDAR(self, data):
        # Check if there are obstacles in front
        front_laser = data.ranges[len(data.ranges) // 2]
        if front_laser < 0.2:
            # Stop and change direction
            self.change_robot_speeds(0.0, 0.0)
            self.angular_speed = -self.angular_speed  # Change direction
        else:
            # Move forward or backward based on the angular speed
            self.change_robot_speeds(self.linear_speed, self.angular_speed)

    def processCollisions(self, data):
        # If a collision occurs, stop and change direction
        self.change_robot_speeds(0.0, 0.0)
        self.angular_speed = -self.angular_speed  # Change direction

def main(args=None):
    rclpy.init()
    
    left_right_robot = LeftRightRobot()

    rclpy.spin(left_right_robot)

    left_right_robot.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
