import rospy
from time import time
from geometry_msgs.msg import Twist

class PIDController:


    def __init__(self,set_point):
        print("Initializing PIDController")

        self.Kp = 1
        self.Ki = 0
        self.Kd = 15
        self.SET_POINT = set_point
        self.previous_time = 0
        self.last_error = 0
        self.integral = 0
    
    def GetPV(self, input_value):
        PV_UPPER_BOUND = 2
        PV_LOWER_BOUND = PV_UPPER_BOUND * -1

        error = self.SET_POINT - input_value
        self.integral += error

        kp = self.Kp * error
        ki = self.Ki * self.integral
        kd = self.Kd * (error - self.last_error)

        pv = kp + ki + kd
        
        if pv > PV_UPPER_BOUND:
            pv = PV_UPPER_BOUND
        elif pv < PV_LOWER_BOUND:
            pv = PV_LOWER_BOUND

        print ' %f * %f + %f * %f + %f * %f' %(self.Kp, error, self.Ki, self.integral,self.Kd, (error - self.last_error))
        print ' distance: %f ,error: %f angular: %f ' %(input_value, error, pv)

        self.last_error = error
        return pv
