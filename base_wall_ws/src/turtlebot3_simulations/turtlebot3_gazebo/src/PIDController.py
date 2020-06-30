import rospy
from time import time
from geometry_msgs.msg import Twist

class PIDController:


    def __init__(self,set_point):
        print("Initializing PIDController")

        self.error_sample_array = []
        self.derivative_sample_array = []

        self.Kp = 5
        self.Ki = 0
        self.Kd = 15
        self.SET_POINT = set_point
        self.previous_time = 0
        self.last_error = 0
        self.cummulated_error = 0
    
    def calculate_speed(self,input_value):
        output_speed = Twist()

        #filter to average the sensor value
        self.error_sample_array.append(input_value)

        if len(self.error_sample_array) > 10:
            del self.error_sample_array[0]
        
        filtered_error = sum(self.error_sample_array)/len(self.error_sample_array)

        current_time = time()
        elapsed_time = current_time - self.previous_time

        #PID controller
        error = self.SET_POINT - filtered_error

        self.cummulated_error += error * elapsed_time
        error_derivative = (error - self.last_error)/elapsed_time

        #Filter deritive value 
        self.derivative_sample_array.append(error_derivative)

        if len(self.derivative_sample_array) > 20:
            del self.derivative_sample_array[0]
        
        filtered_derivative_value = sum(self.derivative_sample_array)/len(self.derivative_sample_array)

        output_angular = self.Kp * error + self.Ki * self.cummulated_error + self.Kd * error_derivative
        output_linear = min(max((0.15 - (2 * abs(error))),0.02),0.1)

        robot_linear_speed = output_linear
        robot_angualr_speed = min(max(output_angular,-1.3),1.3)

        output_speed.linear.x = robot_linear_speed
        output_speed.angular.z = robot_angualr_speed

        self.last_error = error
        self.previous_time = current_time

        print ' %f * %f + %f * %f + %f * %f' %(self.Kp, error, self.Ki, self.cummulated_error,self.Kd,error_derivative)
        print ' distance: %f ,error: %f angular: %f linear: %f' %(input_value, error, robot_angualr_speed, robot_linear_speed)

        return output_speed