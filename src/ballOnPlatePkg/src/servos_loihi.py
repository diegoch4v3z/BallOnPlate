#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os
from typing import Any
from std_msgs.msg import Float32MultiArray
import Adafruit_PCA9685
import rospy, time, datetime
import numpy as np
from plots import plotTwoAxis, saveArray, saveArrayLQR, saveTimeArrayServos, saveArrayACC

class servos: 
    def __init__(self): 
        rospy.init_node('Servo', anonymous=True)
        self.start_time = rospy.Time.now() 
        self.t = rospy.Time.now()
        self.tn = np.array([])
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=2)
        self.pwm.set_pwm_freq(60)
        self.pwm.set_pwm(0, 0, 375)
        self.pwm.set_pwm(1, 0, 375)

        self.i = 0 
        self.plot = True
    def __call__(self, servoNo, pwmVal, randVal = 0):
        self.pwm.set_pwm(servoNo, randVal, pwmVal)
    
    def initNode(self): 

        self.sub = rospy.Subscriber('servoData', Float32MultiArray, callback=self.callback, queue_size=1)    
        self.rate = rospy.Rate(240)
    def callback(self, msg): 
        rospy.loginfo("Received Servo Control: %s", msg.data)
        
            

        #
    def runNode(self):
        try: 
            while not rospy.is_shutdown():
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    
     
if __name__ == '__main__':
    s = servos()
    s.initNode()
    s.runNode()