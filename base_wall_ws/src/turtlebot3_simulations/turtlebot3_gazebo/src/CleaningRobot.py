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

        self.FOLLOW_WALL_SET_POINT              = 0.40
        self.FOLLOW_WALL_SET_POINT_START        = 0.35
        self.FOLLOW_WALL_SET_POINT_END          = 0.50
        self.OBSTACLE_DETECTION_SET_POINT_FAR   = 1.00
        self.STUCK_DETECTION_DISTANCE           = 0.20
        self.STUCK_MAX_COUNTER                  = 30

        self.PID_controller                     = PIDController(self.FOLLOW_WALL_SET_POINT)
        self.current_speed                      = Twist()
        self.cur_command                        = "start"
        self.previous_searching_time            = 0
        self.state                              = 0
        self.previous_state                     = 0
        self.robot_stuck_counter                = 0

        self.state_desc = {
            0: 'Idle',
            1: 'Finding wall',
            2: 'Following wall',
            3: 'Stuck',
            4: 'Turning'
        }

        self.directions = {
            'left':10,
            'fleft':10,
            'front1':10,
            'front2':10,
            'front':10,
            'fright':10,
            'right':10,
        }

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

    def _wall_at_right(self,distance_set_point):

        distance = min(self.directions['right'], self.directions['fright'])
        return distance - distance_set_point <= 0

    def _wall_at_front(self,distance_set_point):

        distance = self.directions['front'] 
        return distance - distance_set_point <= 0

    def _wall_at_left(self,distance_set_point):

        distance = min(self.directions['left'], self.directions['fleft'])
        return distance - distance_set_point <= 0


    def wall_is_found(self,distance_set_point):

        return self._wall_at_right(distance_set_point) or self._wall_at_front(distance_set_point) or self._wall_at_left(distance_set_point)

    def _is_in_corner(self):
        if self.cleaner_side == "right":
            if self.directions["right"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["fright"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["front1"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["front2"] < self.FOLLOW_WALL_SET_POINT\
            and not self.directions["left"] < self.FOLLOW_WALL_SET_POINT:
                return True
        if self.cleaner_side == "left":
            if self.directions["left"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["fleft"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["front1"] < self.FOLLOW_WALL_SET_POINT\
            and self.directions["front2"] < self.FOLLOW_WALL_SET_POINT\
            and not self.directions["right"] < self.FOLLOW_WALL_SET_POINT:
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

    def vortex_searching(self,increment_delay,search_speed_increment,search_speed_max):
        current_time = time()

        if current_time - self.previous_searching_time >= increment_delay:
            self.current_speed.linear.x += search_speed_increment
            self.previous_searching_time = current_time

        self.current_speed.angular.z = 0.2
        self.current_speed.linear.x = min(self.current_speed.linear.x,search_speed_max)


    def finding_wall(self,distance_set_point):
        #detecting wall position
        #act based on the position
        """
        if self._wall_at_front(distance_set_point):
            #at far front, approaching slowly
            self.current_speed.linear.x  = 0.1
            self.current_speed.angular.z = 0     

        elif self._wall_at_right(distance_set_point):
            #at far right, rotate right
            self.current_speed.linear.x  = 0
            self.current_speed.angular.z = -0.1

        elif self._wall_at_left(distance_set_point):
            #at far left, rotate left
            self.current_speed.linear.x  = 0
            self.current_speed.angular.z = 0.1  
        """
        if self._is_in_corner():
            self.state = STATES["TURNING"]
        else:
            #no obstacles, vortex searching
            self.vortex_searching(10,0.05,1.0)

    def state_work(self):

        if self.cur_command == "stop":
            print '\nstop command received, to idle'
            self.current_speed.linear.x = 0
            self.current_speed.angular.z = 0
            self.publisher.publish(self.current_speed) 
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
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.publisher.publish(self.current_speed)
                self.cur_command = "stop"
                self.state = STATES["STUCK"]
                self._sendStatusMessage(self.state_desc[self.state])
                self.photographer.CaptureImage()

            elif self.wall_is_found(self.FOLLOW_WALL_SET_POINT_START):
                #change to follow wall
                print '\nto follow wall' 
                self.state = STATES["FOLLOW_WALL"]
                self._sendStatusMessage(self.state_desc[self.state])
            else:
                print '\nfinding wall' 
                self.finding_wall(self.OBSTACLE_DETECTION_SET_POINT_FAR)
                self.publisher.publish(self.current_speed)
                #print '\nfleft: %f ,front: %f ,fright: %f ' %(self.directions['front'], self.directions['fleft'], self.directions['fright'])

        #follow wall state
        elif self.state == STATES["FOLLOW_WALL"]:
            if self.robot_is_stuck():
                print '\nrobot is stuck, to idle' 
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.cur_command = "stop"
                self.publisher.publish(self.current_speed)
                self.state = STATES["STUCK"]
                self._sendStatusMessage(self.state_desc[self.state])
                self.photographer.CaptureImage()
            elif self._is_in_corner():
                print "In a corner"
                self.state = STATES["TURNING"]
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 1 if self.cleaner_side == "right" else -1
                self.publisher.publish(self.current_speed)
                self._sendStatusMessage(self.state_desc[self.state])
            else:
                print '\nfollowing wall' 
                if self.cleaner_side == "right":
                    distance_to_wall = min(self.directions['right'],self.directions['fright'],self.directions['front1'])
                elif self.cleaner_side == "left":
                    distance_to_wall = min(self.directions['left'],self.directions['fleft'],self.directions['front2'])

                self.current_speed = self.PID_controller.GetPV(distance_to_wall)

                if self.cleaner_side == "left":
                    self.current_speed.angular.z = self.current_speed.angular.z * -1

                self.publisher.publish(self.current_speed)

        elif self.state == STATES["STUCK"]:
            if self.cur_command == "start":
                state = STATES["IDLE"]
                self._sendStatusMessage(self.state_desc[self.state])

        elif self.state == STATES["TURNING"]:
            if not self._is_in_corner():
                self.state = STATES["FOLLOW_WALL"]
                self._sendStatusMessage(self.state_desc[self.state])


        else:
            self.current_speed.linear.x = 0
            self.current_speed.angular.z = 0
            self.publisher.publish(self.current_speed)
            self.state = STATES["IDLE"]
            print '\nunknown state, to idle' 
            self._sendStatusMessage(self.state_desc[self.state])

        print '\nleft: %f ,front: %f ,right: %f ' %(self.directions['left'],self.directions['front'],self.directions['right'])
        print '\nlinear: %f ,angular: %f ' %(self.current_speed.linear.x, self.current_speed.angular.z )
