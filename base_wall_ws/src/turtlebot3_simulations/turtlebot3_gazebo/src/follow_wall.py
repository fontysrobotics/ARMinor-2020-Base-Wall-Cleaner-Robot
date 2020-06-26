#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time

from CleaningRobot import CleaningRobot

from WebCommunication import Photographer
from WebCommunication import WebController

def main():

    global cur_command

    cur_command = "start"
    robot_is_stuck = False

    rospy.init_node('follow_wall')
    photographer = Photographer()
    webcontroller = WebController()
    test_robot = CleaningRobot(webcontroller, photographer)

    rate = rospy.Rate(10)

    cur_command = None

    while not rospy.is_shutdown():
        cur_command = webcontroller.GetCommand()

        if cur_command == "start":
            test_robot.state_work()
        else:
            continue
        
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()

