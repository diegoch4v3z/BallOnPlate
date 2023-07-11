#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
from typing import Any 
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from constants import Constants

class PIDClass: 
    def __init__(self): 
        self.x = 0
        self.y = 0

        ## Import Contansts
        c = Constants()
        self.kPID = c.PIDConstants()

        ## 
        self.Ix = np.array([1, 1])
        self.Iy = np.array([1, 1])
        self.SP = np.array([0, 0])
        self.Ux = np.array([0, 1])
        self.Uy = np.array([0, 1])


    def initNode(self): 
        rospy.init_node('PID', anonymous=True)
        self.rate = rospy.Rate(240)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
    def runNode(self):
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 

    def callback(self, msg): 
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        self.SP = np.append(self.SP, self.kPID[6])
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
        ux = self.pidFunction(self.SP[-1],self.Ix[-1], self.Ix[-2], self.kPID[9], self.kPID[10],
                              self.kPID[2], self.kPID[1], self.kPID[0])
        uy = self.pidFunction(self.SP[-1],self.Iy[-1], self.Iy[-2], self.kPID[9], self.kPID[10],
                              self.kPID[5], self.kPID[4], self.kPID[3])
        Ux = np.append(self.Ux, ux)
        Uy = np.append(self.Uy, uy)
        servoData = Float32MultiArray()
        servoData.data = [ux, uy]
        self.pubServo.publish(servoData)
        self.rate.sleep()
        
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]
    def pidFunction(self, sp, cv, pv, iErr, dt, kD, kI, kP): 
        # K values 
        error = sp-pv 
        iErr = iErr + kI*error*dt
        dErr = (cv - pv)/dt 
        u = kP*error + kI*iErr - kD*dErr
        return u 
        

if __name__ == '__main__':
    P = PIDClass()
    P.initNode()
    P.runNode()
    

    