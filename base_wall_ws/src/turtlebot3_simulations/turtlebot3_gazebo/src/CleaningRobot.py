import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time

from PIDController import PIDController

class CleaningRobot:
    def __init__(self):
        print("Initializing CleaningRobot")

        self.PID_controller = PIDController()
        self.cur_command = "start"
        self.state = 0
        self.previous_state = 0
        self.state_desc = {
            0: 'idle',
            1: 'finding the wall',
            2: 'follow the wall'
        }

        self.directions = {
            'left':0,
            'fleft':0,
            'front1':0,
            'front2':0,
            'front':0,
            'fright':0,
            'right':0,
        }

        self.distance_threshold = 0.40

        self.robot_stuck_counter = 0

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size =1 )
        self.subscriber = rospy.Subscriber('/scan',LaserScan, self.callback_laser_scan)

    def callback_laser_scan(self,msg):
        self.directions = {
            'right':min(min(msg.ranges[270:305]),10),
            'fright':min(min(msg.ranges[306:341]),10),
            'front1':min(min(msg.ranges[342:360]),10),
            'front2':min(min(msg.ranges[0:17]),10),
            'front':min(min(min(msg.ranges[342:360]),min(msg.ranges[0:17])),10),
            'fleft':min(min(msg.ranges[18:53]),10),
            'left':min(min(msg.ranges[54:90]),10),
        }

    def state_work(self):
        robot_speed = Twist()
        #robot_speed.linear.x=0
        #robot_speed.angular.z=0

        #idle state
        if self.state == 0:
            if self.cur_command == "start":
                #start finding wall
                print '\nstart command received to find wall' 
                self.state = 1

        
        #find wall state
        elif self.state == 1:
            if self.cur_command == "stop":
                #change to idle
                self.state = 0
                print '\nstop command received, to idle' 
            elif self.directions['front'] > self.distance_threshold and self.directions['fleft'] > self.distance_threshold and self.directions['fright'] < self.distance_threshold:
                #change to follow wall
                print '\nto follow wall' 
                self.state = 2
            else:
                print '\nfinding wall' 
                robot_speed.linear.x = 0.1
                robot_speed.angular.z = -0.5
                print '\nfleft: %f ,front: %f ,fright: %f ' %(self.directions['front'], self.directions['fleft'], self.directions['fright'])
                self.publisher.publish(robot_speed)

        #follow wall state
        elif self.state == 2:
            if self.cur_command == "stop":
                #change to idle
                print '\nstop command received, to idle' 
                self.state = 0
            
            else:
                print '\nfollowing wall' 
                distance_to_wall = min(self.directions['right'],self.directions['fright'],self.directions['front1'])
                robot_speed = self.PID_controller.calculate_speed(distance_to_wall)
                self.publisher.publish(robot_speed)

        else:
            print '\nunknown state' 
            x = 5
