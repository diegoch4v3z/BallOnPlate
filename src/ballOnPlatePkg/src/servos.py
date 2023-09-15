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
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
        self.pwm.set_pwm_freq(60)
        self.pwm.set_pwm(0, 0, 375)
        self.pwm.set_pwm(1, 0, 375)
        self.datauxPlot = np.array([0, 0])
        self.datauyPlot = np.array([0, 0])
        self.dataServoXPlot = np.array([0, 0])
        self.dataServoYPlot = np.array([0, 0])
        self.timeSeries = np.array([0, 0])
        self.disturbance = np.array([0, 0])

        self.i = 0 
        self.plot = True
    def __call__(self, servoNo, pwmVal, randVal = 0):
        self.pwm.set_pwm(servoNo, randVal, pwmVal)
    
    def initNode(self): 

        self.sub = rospy.Subscriber('servoData', Float32MultiArray, callback=self.callback)
        self.rate = rospy.Rate(240)
        self.start_time = rospy.Time.now()
    
    def callback(self, msg): 
        self.i = self.i + 1
        self.datauxPlot = np.append(self.datauxPlot, msg.data[0])
        self.datauyPlot = np.append(self.datauyPlot, msg.data[1])
        #self.uy = self.mappingUy(msg.data[1])
        self.ux = self.mappingUx(msg.data[0])
        self.uy = self.mappingUy(msg.data[1])
        self.dataServoXPlot = np.append(self.dataServoXPlot, self.ux)
        self.dataServoYPlot = np.append(self.dataServoYPlot, self.uy)
        if self.i == 1300 or self.i == 3500: 
            
            self.pwm.set_pwm(1, 0, 450)
            rospy.sleep(0.1)
            print('Disturbance')
            self.disturbance = np.append(self.disturbance, np.array([0.25]))
        else:
            self.pwm.set_pwm(0, 0, self.ux)
            self.pwm.set_pwm(1, 0, self.uy)
            self.disturbance = np.append(self.disturbance, np.array([0]))
        self.current_time = (rospy.Time.now() - self.start_time).to_sec()
        self.timeSeries = np.append(self.timeSeries, self.current_time)
        
            

        # 

    def runNode(self):
        try: 
            while not rospy.is_shutdown():
                self.rate.sleep()
                #print('Hello')
            if self.plot: 
                saveArrayACC(self.datauxPlot, self.datauyPlot, self.timeSeries, 'controlSignalData')
                saveArrayACC(self.dataServoXPlot, self.dataServoYPlot, self.timeSeries, 'servoSignalData')
                saveTimeArrayServos(self.timeSeries, 'ServoTiming')
                saveArrayACC(self.disturbance, self.disturbance, self.timeSeries, 'disturbanceData')
                #plotTwoAxis(self.datauxPlot, self.datauyPlot, self.timeSeries, 'Control Signal', 'Time (s)', 'Control Signal Value', 'controlSignal')
                #plotTwoAxis(self.dataServoXPlot, self.dataServoYPlot, self.timeSeries, 'Mapped Signal Servo', 'Time (s)', 'Servo Signal', 'servoSignal', limit=False) 
        except rospy.ROSInterruptException: 
            pass 

    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]

    def uSafety(self, u):
        if u > 1.1:
            u = 1
        elif u < -1.1: 
            u = -1
        return u
    def mappingUx(self, u): 
        if u >= 0:
            servoX = int(-1*115*u+375)
        elif u < 0:
            servoX = int(-1*125*u+375)
        else: 
            servoX = 375
        #Servo Security Check 
        if(servoX >= 500): 
            servoX = 500
        elif(servoX < 260): 
            servoX = 260
        return servoX
    def mappingUy(self, u): 
        if u >= 0:
            servoY = int(-1*115*u+375)
        elif u < 0:
            servoY = int(-1*125*u+375)
        else: 
            servoY = 375
        if(servoY >= 500): 
            servoY = 500
        elif(servoY < 260): 
            servoY = 260
        return servoY

    
     
if __name__ == '__main__':
    s = servos()
    s.initNode()
    s.runNode()