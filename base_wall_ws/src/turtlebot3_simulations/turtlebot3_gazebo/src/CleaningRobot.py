import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time

from PIDController import PIDController

class CleaningRobot:
    def __init__(self):
        print("Initializing CleaningRobot")

        self.FOLLOWING_DISTANCE_SET_POINT       = 0.40
        self.OBSTACLE_DETECTION_SET_POINT_CLOSE = 0.35
        self.OBSTACLE_DETECTION_SET_POINT_FAR   = 0.50
        self.STUCK_DETECTION_DISTANCE           = 0.20
        self.STUCK_MAX_COUNTER                  = 30

        self.PID_controller                     = PIDController(self.FOLLOWING_DISTANCE_SET_POINT)
        self.current_speed                      = Twist()
        self.cur_command                        = "start"
        self.previous_searching_time            = 0
        self.state                              = 0
        self.previous_state                     = 0
        self.robot_stuck_counter                = 0

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

        self.current_speed.linear.x = min(self.current_speed.linear.x,search_speed_max)


    def finding_wall(self,distance_set_point):
        #detecting wall position
        #act based on the position
        if self._wall_at_right(distance_set_point):
            #at far right, rotate right
            self.current_speed.linear.x  = 0
            self.current_speed.angular.z = -0.1

        elif self._wall_at_front(distance_set_point):
            #at far front, approaching slowly
            self.current_speed.linear.x  = 0.05
            self.current_speed.angular.z = 0          

        elif self._wall_at_left(distance_set_point):
            #at far left, rotate left
            self.current_speed.linear.x  = 0
            self.current_speed.angular.z = 0.1  
        else:
            #no obstacles, vortex searching
            vortex_searching(10,0.1,1.0)

    def state_work(self):

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
                print '\nstop command received, to idle'
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.publisher.publish(self.current_speed) 
                self.state = 0

            
            elif self.robot_is_stuck():
                print '\nrobot is stuck, to idle' 
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.publisher.publish(self.current_speed)
                self.state = 0

            elif self.wall_is_found():
                #change to follow wall
                print '\nto follow wall' 
                self.state = 2
            else:
                print '\nfinding wall' 
                self.finding_wall(OBSTACLE_DETECTION_SET_POINT_FAR)
                self.publisher.publish(self.current_speed)
                #print '\nfleft: %f ,front: %f ,fright: %f ' %(self.directions['front'], self.directions['fleft'], self.directions['fright'])

        #follow wall state
        elif self.state == 2:
            if self.cur_command == "stop":
                #change to idle
                print '\nstop command received, to idle' 
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.publisher.publish(self.current_speed)
                self.state = 0

            elif self.robot_is_stuck():
                print '\nrobot is stuck, to idle' 
                self.current_speed.linear.x = 0
                self.current_speed.angular.z = 0
                self.publisher.publish(self.current_speed)
                self.state = 0

            else:
                print '\nfollowing wall' 
                distance_to_wall = min(self.directions['right'],self.directions['fright'],self.directions['front1'])
                self.current_speed = self.PID_controller.calculate_speed(distance_to_wall)
                self.publisher.publish(self.current_speed)

        else:
            self.current_speed.linear.x = 0
            self.current_speed.angular.z = 0
            self.publisher.publish(self.current_speed) 
            self.state = 0
            print '\nunknown state, to idle' 

