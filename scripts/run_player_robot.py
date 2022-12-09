#!/usr/bin/env python
import time
from robot_class import Robot
import rospy

def main(args=None):
    rospy.init_node("PlayerRobot",anonymous=True)
    player_robot = Robot("ponce")
    time.sleep(2) #Sleep to allow time to initialize. Otherwise subscriber might recieve an empty message
    player_robot.main_loop()

if __name__ == '__main__':
    main()