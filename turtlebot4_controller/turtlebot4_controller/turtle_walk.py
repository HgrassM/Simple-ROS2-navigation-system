#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

import math

class WalkNode(Node):
    #Class constructor with all the needed  publishers and receivers instantiated

    def __init__(self):
        super().__init__("turtle_walk")

        self.laser_info = self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data)
        
        self.pose_sub = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)

        self.velocity_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.velocity_param = Twist()
        
        self.pose = None
        self.quaternion = None

        self.setpoint = None

        self.x_sum = 0.0
        self.z_sum = 0.0

        self.front_view_range = None
        
        self.obstacle_flag = False
        self.flag = True
    
    #This method calculates all the coordinates where an obstacle is detected
    #Keep in mind that those coordinates are calculated using the robot's position as the (0,0) coordinate
    
    def calculate_obstacle_distance(self, angle_min, angle_increment, ranges):
        self.get_logger().info("Calculating obstacles distances from spawn point...")
        
        angle_arc_increment = angle_min + (angle_increment)

        for distance in ranges:
            if not (math.isinf(distance)):
                dy = math.sin(angle_arc_increment)*distance
                dx = math.cos(angle_arc_increment)*distance
                
                self.get_logger().info(f"Object detected at coordinate ({dx}, {dy})")

            angle_arc_increment = angle_arc_increment + angle_increment         

    #This method calculates the front view angle of the robot

    def calculate_laser_arc(self, angle_increment) -> tuple:
        i = 0
        min_index = 0

        while (i < (math.pi/4)):
            i = i + angle_increment
            min_index = min_index + 1

        max_index = (min_index*3)

        return (min_index, max_index)
    
    #This method is responsible for handling the "scan" msgs and publishing on "cmd_vel" based on the received data

    def laser_callback(self, msg: LaserScan):
        if (self.flag):
            self.calculate_obstacle_distance(msg.angle_min, msg.angle_increment, msg.ranges)

            self.front_view_range = self.calculate_laser_arc(msg.angle_increment)
            range_in_degrees = (self.front_view_range[1] - self.front_view_range[0])*(msg.angle_increment)

            self.flag = False
            self.get_logger().info(f"The front view is a {range_in_degrees} rad arc that goes from range[{self.front_view_range[0]}] to range[{self.front_view_range[1]}]")
            self.define_target()
        else:
            min_distance = min(msg.ranges[self.front_view_range[0]:self.front_view_range[1]])
            
            if (min_distance > 0.5 and self.pose != None and self.quaternion != None and self.obstacle_flag == False):
                self.pid_walk(self.setpoint)

            elif (self.obstacle_flag and min_distance > 0.5):
                self.walk_linear_fixed()

                if (min(msg.ranges) > 0.5):
                    self.obstacle_flag = False

            else:                
                self.obstacle_flag = True
                
                front_range = msg.ranges[self.front_view_range[0]:self.front_view_range[1]]
                min_distance = (len(front_range)//2) - 1

                if (min_distance in front_range[0:min_distance]):
                    self.rotate_angular_fixed(1.0)
                else:
                    self.rotate_angular_fixed(-1.0)

    def pose_callback(self, msg: Odometry):
        self.pose = msg.pose.pose.position
        self.quaternion = msg.pose.pose.orientation

    #This method calculates the velocity of the robot to the target using a PID controller

    def pid_walk(self, setpoint):
        KP = 0.3
        KI = 0.1
    
        x_error = math.sqrt(((setpoint[0] - self.pose.x)**2) + (setpoint[1] - self.pose.y)**2)

        setpoint_angle = math.atan2(setpoint[1] - self.pose.y, setpoint[0] - self.pose.x)
        pos_angle = math.atan2(2.0*(self.quaternion.y*self.quaternion.x + self.quaternion.w*self.quaternion.z), 1.0 - 2.0*(self.quaternion.z**2 + self.quaternion.y**2))

        z_error = setpoint_angle - pos_angle

        self.x_sum += x_error
        self.z_sum += z_error
          
        if (abs(z_error) > 0.3):
            self.velocity_param.linear.x = 0.0
            self.velocity_param.angular.z = (KP*z_error) + (KI*self.z_sum)
       
            self.velocity_pub.publish(self.velocity_param)

        elif (x_error > 0.1):
            self.velocity_param.linear.x = (KP*x_error) + (KI*self.x_sum)
            self.velocity_param.angular.z = 0.0
            
            self.velocity_pub.publish(self.velocity_param)
        else:
            self.velocity_param.linear.x = 0.0
            self.velocity_param.angular.z = 0.0
            
            self.velocity_pub.publish(self.velocity_param)

            self.define_target()
           
    #These methods are used when the robot needs to dodge obstacles

    def rotate_angular_fixed(self, velocity):
        self.velocity_param.linear.x = 0.0
        self.velocity_param.angular.z = velocity
        
        self.velocity_pub.publish(self.velocity_param)

    def walk_linear_fixed(self):
        self.velocity_param.linear.x = 1.0
        self.velocity_param.angular.z = 0.0
        
        self.velocity_pub.publish(self.velocity_param)

    def define_target(self):
        
        if (self.pose != None):
            print(f"Current coordinate is ({self.pose.x}, {self.pose.y})")

        print("\nType the x coordinate of the desired location:")
        x = float(input())

        while (type(x) != type(1.0)):
            print("Invalid input! The number must be a float!")
            x = float(input())
        
        print("Type the y coordinate of the desired location:")
        y = float(input())

        while (type(y) != type(1.0)):
            print("Invalid input! The number must be a float!")
            y = float(input())

        self.setpoint = (x, y)

def main(args=None):
    rclpy.init(args=args)
    node = WalkNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
