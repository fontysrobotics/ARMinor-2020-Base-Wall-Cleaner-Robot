#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time


from WebCommunication import Photographer
from WebCommunication import WebController


directions = {
        'left':0,
        'fleft':0,
        'front1':0,
        'front2':0,
        'front':0,
        'fright':0,
        'right':0,
}

state = 0
state_desc = {
    0: 'finding the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'robot stuck'
}

state_description = ''

distance_threshold = 1.0
distance_to_wall_parameter = 0.40

lidar_sensor_sample_array = []
derivative_error_sample_array = []

Kp = 5
Ki = 0
Kd = 15
previous_time = 0
last_error = 0
cummulated_error = 0

robot_stuck_counter =0

def callback_laser_scan(msg):
    global directions
    directions = {
        'right':min(min(msg.ranges[270:305]),10),
        'fright':min(min(msg.ranges[306:341]),10),
        'front1':min(min(msg.ranges[342:360]),10),
        'front2':min(min(msg.ranges[0:17]),10),
        'front':min(min(min(msg.ranges[342:360]),min(msg.ranges[0:17])),10),
        'fleft':min(min(msg.ranges[18:53]),10),
        'left':min(min(msg.ranges[54:90]),10),
    }


def change_state(new_state):
    global state, state_desc
    if new_state is not state:
        print 'state - [%s] - %s , %s ' %(new_state,state_desc[new_state], state_description)
        state = new_state

def determine_and_change_state():
    global directions
    global state_description
    global state

    robot_speed = Twist()
    linear_x = 0
    angular_z = 0

    robot_is_stuck = determine_robot_stuck()
    
    if robot_is_stuck == True:
        state_description = 'robot stuck'
        change_state(3)

    elif directions['front'] > distance_threshold and directions['fleft'] > distance_threshold and directions['fright'] > distance_threshold:
        state_description = 'no wall'
        change_state(0)

    elif directions['front'] < distance_threshold and directions['fleft'] > distance_threshold and directions['fright'] > distance_threshold:
        state_description = 'wall at front'
        change_state(1)

    elif directions['front'] > distance_threshold and directions['fleft'] > distance_threshold and directions['fright'] < distance_threshold:
        state_description = 'wall at front right'
        change_state(2)

    elif directions['front'] > distance_threshold and directions['fleft'] < distance_threshold and directions['fright'] > distance_threshold:
        state_description = 'wall at front left'
        change_state(0)

    elif directions['front'] < distance_threshold and directions['fleft'] > distance_threshold and directions['fright'] < distance_threshold:
        state_description = 'front and front right'
        change_state(1)

    elif directions['front'] < distance_threshold and directions['fleft'] < distance_threshold and directions['fright'] > distance_threshold:
        state_description = 'front and front left'
        change_state(1)

    elif directions['front'] < distance_threshold and directions['fleft'] < distance_threshold and directions['fright'] < distance_threshold:
        state_description = 'front front-left and front-right'
        change_state(1)

    elif directions['front'] > distance_threshold and directions['fleft'] > distance_threshold and directions['fright'] > distance_threshold:
        state_description = 'front-left and front-right'
        change_state(0)

    else:
        state_description = 'unknown'
#To find a wall, the robot simply turns to right until the lidar found a obstacle        
def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.5
    return msg

#Turn to the left to keep the wall(obstacle) to the right side of the robot
def turn_left():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.5
    return msg

#Follow the obstacle(wall), keep a distance to the obstacle and adjust angular as well as linear speed based on the robot's distance to the wall
def follow_the_wall():
    global directions
    global distance_to_wall_parameter
    global previous_time
    global last_error
    global cummulated_error
    global lidar_sensor_sample_array
    global derivative_error_sample_array

    
    distance_to_wall = min(directions['right'],directions['fright'],directions['front1'])
    msg = Twist()

    #filter to average the sensor value
    lidar_sensor_sample_array.append(distance_to_wall)

    if len(lidar_sensor_sample_array) > 10:
        del lidar_sensor_sample_array[0]
    
    filtered_distance = sum(lidar_sensor_sample_array)/len(lidar_sensor_sample_array)

    current_time = time()
    elapsed_time = current_time - previous_time

    #PID controller
    error = distance_to_wall_parameter - filtered_distance

    cummulated_error += error * elapsed_time
    rate_error = (error - last_error)/elapsed_time

    #Filter deritive value 
    derivative_error_sample_array.append(rate_error)

    if len(derivative_error_sample_array) > 20:
        del derivative_error_sample_array[0]
    
    filtered_rate_error = sum(derivative_error_sample_array)/len(derivative_error_sample_array)

    output_angular = Kp * error + Ki * cummulated_error + Kd * rate_error
    output_linear = min(max((0.15 - (2 * abs(error))),0.02),0.1)

    robot_linear_speed = output_linear
    robot_angualr_speed = min(max(output_angular,-1.3),1.3)

    msg.linear.x = robot_linear_speed
    msg.angular.z = robot_angualr_speed

    last_error = error
    previous_time = current_time
    
    #print out to debug
    #print '\ndistance: %f ,error: %f angular: %f linear: %f' %(filtered_distance, error, robot_angualr_speed, robot_linear_speed)
    #print ' %f * %f + %f * %f + %f * %f' %(Kp, error, Ki, cummulated_error,Kd,rate_error)
    #print ' %f * %f + %f * %f + %f * %f' %(Kp, error, Ki, cummulated_error,Kd,filtered_rate_error)

    return msg

def robot_stuck():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    return msg

def determine_robot_stuck():
    global directions
    global state_description
    global robot_stuck_counter

    minimum_distance = min(directions.values())

    #print '\nstuck: %f , min: %f ' %(robot_stuck_counter,minimum_distance)

    if minimum_distance <= 0.20:
        robot_stuck_counter += 1
    else:
        robot_stuck_counter = 0

    if robot_stuck_counter >= 30:
        
        return True
    
    return False

def main():

    robot_is_stuck = False

    rospy.init_node('follow_wall')
    photographer = Photographer()
    webcontroller = WebController()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size =1 )
    sub = rospy.Subscriber('/scan',LaserScan, callback_laser_scan)

    rate = rospy.Rate(10)

    cur_command = None

    photographer.CaptureImage()

    while not rospy.is_shutdown():
        cur_command = webcontroller.GetCommand()
        if cur_command == "stop":
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            cur_command = "none"
            pub.publish(msg)
        elif cur_command == "start":
            determine_and_change_state()
            msg = Twist()
            if state == 0:
                msg = find_wall()
            elif state == 1:
                msg = follow_the_wall()
            elif state == 2:
                msg = follow_the_wall()
            elif state == 3:
                msg = robot_stuck()
            else:
                rospy.logerr('Unknown state')
            pub.publish(msg)

        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()

