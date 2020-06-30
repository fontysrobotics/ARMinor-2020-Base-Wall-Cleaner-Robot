import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time

from PIDController import PIDController

STATES = {
    'IDLE': 0,
    'FIND_WALL': 1,
    'FOLLOW_WALL': 2,
    'STUCK': 3,
    'TURNING': 4
}

class CleaningRobot:
    def __init__(self, webcontroller, photographer, cleaner_side="right"):
        print("Initializing CleaningRobot")
        self.webcontroller = webcontroller
        self.photographer = photographer
        self.cleaner_side = cleaner_side

        self.PID_controller = PIDController()
        self.cur_command = "start"
        self.state = 0
        self.previous_state = 0
        self.state_desc = {
            0: 'Idle',
            1: 'Finding wall',
            2: 'Following wall',
            3: 'Stuck',
            4: 'Turning'
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

        self.WALL_DETECTION_DISTANCE = 0.7
        self.CORNER_CHECK_DISTANCE = 0.4

        self.STUCK_DETECTION_DISTANCE = 0.20
        self.STUCK_MAX_COUNTER = 30
        self.robot_stuck_counter = 0

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size =1 )
        self.subscriber = rospy.Subscriber('/scan',LaserScan, self.callback_laser_scan)

    def _sendStatusMessage(self, msg):
        self.webcontroller.SendStatus(msg)

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

    def wall_is_found(self):
        if self.cleaner_side == "right":
            if self.directions['right'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['fright'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['front1'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['front2'] < self.WALL_DETECTION_DISTANCE:
                return True
        elif self.cleaner_side == "left":
            if self.directions['left'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['fleft'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['front1'] < self.WALL_DETECTION_DISTANCE\
            or self.directions['front2'] < self.WALL_DETECTION_DISTANCE:
                return True
        return False

    def _is_in_corner(self):
        if self.cleaner_side == "right":
            if self.directions["right"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["fright"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["front1"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["front2"] < self.WALL_DETECTION_DISTANCE\
            and not self.directions["left"] < self.WALL_DETECTION_DISTANCE:
                return True
        if self.cleaner_side == "left":
            if self.directions["left"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["fleft"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["front1"] < self.WALL_DETECTION_DISTANCE\
            and self.directions["front2"] < self.WALL_DETECTION_DISTANCE\
            and not self.directions["right"] < self.WALL_DETECTION_DISTANCE:
                return True
        return False


    def robot_is_stuck(self):

        minimum_distance = min(self.directions.values())

        print '\nstuck counter: %f , minimun distance: %f ' %(self.robot_stuck_counter,minimum_distance)

        if minimum_distance <= self.STUCK_DETECTION_DISTANCE:
            self.robot_stuck_counter += 1
        else:
            self.robot_stuck_counter = 0

        if self.robot_stuck_counter >= self.STUCK_MAX_COUNTER:
            
            return True
        
        return False

    def state_work(self):
        robot_speed = Twist()

        if self.cur_command == "stop":
            print '\nstop command received, to idle'
            robot_speed.linear.x = 0
            robot_speed.angular.z = 0
            self.publisher.publish(robot_speed) 
            self.state = 0
            self._sendStatusMessage(self.state_desc[self.state])

        #idle state
        if self.state == STATES["IDLE"]:
            if self.cur_command == "start":
                #start finding wall
                print '\nstart command received to find wall' 
                self.state = STATES["FIND_WALL"]
                self._sendStatusMessage(self.state_desc[self.state])

        #find wall state
        elif self.state == STATES["FIND_WALL"]:
            if self.robot_is_stuck():
                print '\nrobot is stuck, to idle' 
                robot_speed.linear.x = 0
                robot_speed.angular.z = 0
                self.publisher.publish(robot_speed)
                self.cur_command = "stop"
                self.state = STATES["STUCK"]
                self._sendStatusMessage(self.state_desc[self.state])
                self.photographer.CaptureImage()

            elif self.wall_is_found():
                #change to follow wall
                print '\nto follow wall' 
                self.state = STATES["FOLLOW_WALL"]
                self._sendStatusMessage(self.state_desc[self.state])
            else:
                print '\nfinding wall' 
                robot_speed.linear.x = 0.1
                robot_speed.angular.z = -0.5
                self.publisher.publish(robot_speed)
                #print '\nfleft: %f ,front: %f ,fright: %f ' %(self.directions['front'], self.directions['fleft'], self.directions['fright'])

        #follow wall state
        elif self.state == STATES["FOLLOW_WALL"]:
            if self.robot_is_stuck():
                print '\nrobot is stuck, to idle' 
                robot_speed.linear.x = 0
                robot_speed.angular.z = 0
                self.cur_command = "stop"
                self.publisher.publish(robot_speed)
                self.state = STATES["STUCK"]
                self._sendStatusMessage(self.state_desc[self.state])
                self.photographer.CaptureImage()
            elif self._is_in_corner():
                print "In a corner"
                self.state = STATES["TURNING"]
                robot_speed.linear.x = 0
                robot_speed.angular.z = 1 if self.cleaner_side == "right" else -1
                self.publisher.publish(robot_speed)
                self._sendStatusMessage(self.state_desc[self.state])
            else:
                print '\nfollowing wall' 
                if self.cleaner_side == "right":
                    distance_to_wall = self.directions["right"]
                elif self.cleaner_side == "left":
                    distance_to_wall = self.directions["left"]

                robot_speed.linear.x = 0.1
                robot_speed.angular.z = self.PID_controller.GetPV(distance_to_wall)

                if self.cleaner_side == "left":
                    robot_speed.angular.z = robot_speed.angular.z * -1

                self.publisher.publish(robot_speed)

        elif self.state == STATES["STUCK"]:
            if self.cur_command == "start":
                state = STATES["IDLE"]
                self._sendStatusMessage(self.state_desc[self.state])

        elif self.state == STATES["TURNING"]:
            if not self._is_in_corner():
                self.state = STATES["FOLLOW_WALL"]
                self._sendStatusMessage(self.state_desc[self.state])


        else:
            robot_speed.linear.x = 0
            robot_speed.angular.z = 0
            self.publisher.publish(robot_speed)
            self.state = STATES["IDLE"]
            print '\nunknown state, to idle' 
            self._sendStatusMessage(self.state_desc[self.state])

