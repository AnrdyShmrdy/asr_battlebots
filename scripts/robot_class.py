#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import random
import time
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from unitysim.msg import BoundingBox3d
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#Angle, Index, and Range notes for laserscan ranges:
#The front laser is the laser that points directly ahead of the front of the robot
#laser at index 90 is likely the front laser. 
#The moment the front of the robot touches a wall, the forward laser indicates a range of 0.5
#Values in range go from 0 to 179
#laser at index 0 is to the right, or clockwise from the front laser
#laser at index 90 is the front laser, which faces directly ahead of the robot
#laser at index 179 is to the left, or counter-clockwise from the front laser
class Robot:
    def __init__(self, robot_name):
        self.cmd_vel_pub = rospy.Publisher("/" + str(robot_name) + "/cmd_vel", Twist, queue_size=10)
        self.healthfinder_sub = rospy.Subscriber("/" + str(robot_name) + "/healthfinder", BoundingBox3d, self.healthfinder_callback)
        self.playerfinder_sub = rospy.Subscriber("/" + str(robot_name) + "/playerfinder", BoundingBox3d, self.playerfinder_callback)
        self.cannon_pub = rospy.Publisher("/" + str(robot_name) + "/cannon", String, queue_size=10)
        self.scan_sub = rospy.Subscriber("/" + str(robot_name) + "/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/" + str(robot_name) + "/odom", PoseWithCovarianceStamped, self.odom_callback)
        self.canshoot_sub = rospy.Subscriber("/" + str(robot_name) + "canshoot", Int32, self.canshoot_callback)
        self.hp_sub = rospy.Subscriber("/" + str(robot_name) + "/hp", Int32, self.hp_callback)
        self.rate = rospy.Rate(10)
        self.is_initialized = False
        self.x_vec_sum = 0
        self.y_vec_sum = 0
        self.angle_increment = None #will be set once node is initialized
        self.angle_min = None #will be set once node is initialized
        self.angle_max = None #will be set once node is initialized
        self.range_max = None #will be set once node is initialized
        self.range_min = None #will be set once node is initialized
        self.ranges = None #will be set once node is initialized
        self.regions = None #will be set once node is initialized
        self.healthfinder_msg = BoundingBox3d()
        self.state = 0
        self.state_linear_speed = 2.0 #linear movement speed when in various states. Does not apply to safe_forward
        self.state_angular_speed = 2.0 #angular movement speed when in various states. Does not apply to safe_forward
        self.state_distance = 1 #distance to compare with the minimum distance of each region in order to determine current state
        self.state_description = ''
        self.center_pos_y = 0.0
        self.size_y = 0.0
        self.time = time.time()
    def set_regions(self):
        self.regions = {
        'right':  min(self.ranges[0:35]),
        'fright': min(self.ranges[36:71]),
        'front':  min(self.ranges[72:107]),
        'fleft':  min(self.ranges[108:143]),
        'left':   min(self.ranges[144:179]),
        }
    def initialize_runtime_variables(self, msg):
        #variables to set once the node is initialized
        #The reason these variables cannot be set in the constructor is because
        #their values can only be determined after the node is initialized
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.ranges = msg.ranges
        self.set_regions()
        self.set_current_state()
    def healthfinder_callback(self, msg):
        self.time = time.time()
        self.healthfinder_msg = msg
    def playerfinder_callback(self, msg):
        pass
    def scan_callback(self, msg):
        if not self.is_initialized:
            self.initialize_runtime_variables(msg)
            print("Variables initialized!")
            self.display_values()
            self.is_initialized = True
        else:
            self.ranges = msg.ranges
            self.set_regions()
            self.set_current_state()   
    def odom_callback(self, msg):
        pass
    def canshoot_callback(self, msg):
        pass
    def hp_callback(self, msg):
        pass
    def get_angle_from_index(self, pos):
        #return the angle (in radians) for the laser at the specified index in laser_msg.ranges
        return pos * self.angle_increment + self.angle_min
    def get_index_from_angle(self, angle_in_radians):
        #return the index in laser_msg.ranges that has the given angle (in radians)
        return int((angle_in_radians - self.angle_min)/self.angle_increment)
    def get_front_laser(self):
        return self.ranges[int(len(self.ranges) / 2)]
    def get_vec_sums(self):
        ranges = self.ranges
        for i in range(len(self.ranges)):
            theta = self.get_angle_from_index(i)
            x_vec = ranges[i] * math.cos(float(theta)) / self.range_max
            y_vec = ranges[i] * math.sin(float(theta)) / self.range_max
            self.x_vec_sum += x_vec
            self.y_vec_sum += y_vec
        self.x_vec_sum = self.x_vec_sum / len(ranges)
        self.y_vec_sum = self.y_vec_sum / len(ranges)
    def safe_forward(self):
        self.get_vec_sums()
        final_angle = math.atan2(self.y_vec_sum,self.x_vec_sum)
        cmdTwist = Twist()
        cmdTwist.linear.x = 10 * self.get_front_laser() / self.range_max
        cmdTwist.angular.z = final_angle
        msg = self.healthfinder_msg
        self.center_pos_y = msg.center.position.y
        self.size_y = msg.size.y
        t0 = time.time()
        if self.center_pos_y < self.size_y and -(self.time - t0) < 1.0:
            print("center pos y: " + str(self.center_pos_y))
            print("size y: " + str(self.size_y))
            print(time.time() - t0)
            cmdTwist.angular.z = self.center_pos_y * -1
        self.cmd_vel_pub.publish(cmdTwist)
    def set_current_state(self):
        if self.regions['front'] > self.state_distance and self.regions['fleft'] > self.state_distance and self.regions['fright'] > self.state_distance:
            self.state_description = 'state 0 - nothing'
            self.state = 0
        elif self.regions['front'] < self.state_distance and self.regions['fleft'] > self.state_distance and self.regions['fright'] > self.state_distance:
            self.state_description = 'state 1 - front'
            self.state = 1
        elif self.regions['front'] > self.state_distance and self.regions['fleft'] > self.state_distance and self.regions['fright'] < self.state_distance:
            self.state_description = 'state 2 - fright'
            self.state = 2
        elif self.regions['front'] > self.state_distance and self.regions['fleft'] < self.state_distance and self.regions['fright'] > self.state_distance:
            self.state_description = 'state 3 - fleft'
            self.state = 3
        elif self.regions['front'] < self.state_distance and self.regions['fleft'] > self.state_distance and self.regions['fright'] < self.state_distance:
            self.state_description = 'state 4 - front and fright'
            self.state = 4
        elif self.regions['front'] < self.state_distance and self.regions['fleft'] < self.state_distance and self.regions['fright'] > self.state_distance:
            self.state_description = 'state 5 - front and fleft'
            self.state = 5
        elif self.regions['front'] < self.state_distance and self.regions['fleft'] < self.state_distance and self.regions['fright'] < self.state_distance:
            self.state_description = 'state 6 - front and fleft and fright'
            self.state = 6
        elif self.regions['front'] > self.state_distance and self.regions['fleft'] < self.state_distance and self.regions['fright'] < self.state_distance:
            self.state_description = 'state 7 - fleft and fright'
            self.state = 7
        else:
            self.state_description = 'unknown state'
    def display_values(self):
        print("self.angle_increment: ", self.angle_increment)
        print("self.angle_min: ", self.angle_min)
        print("self.angle_max: ", self.angle_max)
        print("self.range_max: ", self.range_max)
        print("self.range_min: ", self.range_min)
        print("no of elements in scan ranges: ", int(len(self.ranges)))
        print("self.regions: ", self.regions)
    def turn_left(self):
        msg = Twist()
        msg.angular.z = self.state_angular_speed
        self.cmd_vel_pub.publish(msg)
    def move_back_left(self):
        msg = Twist()
        msg.linear.x = -self.state_linear_speed
        msg.angular.z = -self.state_angular_speed
        self.cmd_vel_pub.publish(msg)
    def move_back_right(self):
        msg = Twist()
        msg.linear.x = -self.state_linear_speed
        msg.angular.z = self.state_angular_speed
        self.cmd_vel_pub.publish(msg)
    def move_front_left(self):
        msg = Twist()
        msg.linear.x = self.state_linear_speed
        msg.angular.z = self.state_angular_speed
        self.cmd_vel_pub.publish(msg)
    def move_front_right(self):
        msg = Twist()
        msg.linear.x = self.state_linear_speed
        msg.angular.z = -self.state_angular_speed
        self.cmd_vel_pub.publish(msg)
    def move_back(self):
        msg = Twist()
        msg.linear.x = -self.state_linear_speed
        self.cmd_vel_pub.publish(msg)
    def move_forward(self):
        msg = Twist()
        msg.linear.x = self.state_linear_speed
        self.cmd_vel_pub.publish(msg)
    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = self.state_linear_speed
        self.cmd_vel_pub.publish(msg)
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.state == 0:
                self.safe_forward()
            if self.state == 1:
                if random.randrange(0, 1) == 1:
                    self.move_back_left()
                else:
                    self.move_back_right()
            if self.state == 2:
                self.move_front_left()
            elif self.state == 3:
                self.move_back_left()
            elif self.state == 4:
                self.move_back_right()
            elif self.state == 5:
                    self.move_back_right()
            elif self.state == 6:
                if random.randrange(0, 1) == 1:
                    self.move_back_left()
                    rospy.Rate(2).sleep()
                else:
                    self.move_back_right()
                    rospy.Rate(2).sleep()
            elif self.state == 7:
                if random.randrange(0, 1) == 1:
                    self.move_back_left()
                    rospy.Rate(2).sleep()
                else:
                    self.move_back_right()
                    rospy.Rate(2).sleep()
            self.rate.sleep()