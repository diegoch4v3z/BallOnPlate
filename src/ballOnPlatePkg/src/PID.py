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
from plots import plotTwoAxis, saveArray

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
        self.IxPlot = np.array([0, 0])
        self.IyPlot = np.array([0, 0])
        self.timeSeries = np.array([0, 0])


    def initNode(self): 
        rospy.init_node('PID', anonymous=True)
        self.rate = rospy.Rate(200)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
        self.start_time = rospy.Time.now()
    def runNode(self):
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 

    def callback(self, msg): 
        self.current_time = (rospy.Time.now()).to_sec()#(rospy.Time.now() - self.start_time).to_sec()
        self.timeSeries = np.append(self.timeSeries, self.current_time)
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        self.SP = np.append(self.SP, self.kPID[6])
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
        ## Add filtered values to plot 
        self.IxPlot = np.append(self.IxPlot, self.Ix[-1])
        self.IyPlot = np.append(self.IyPlot, self.Iy[-1])
        ux = self.pidFunction(self.SP[-1],self.Ix[-1], self.Ix[-2], self.kPID[9], self.kPID[10],
                              self.kPID[2], self.kPID[1], self.kPID[0])
        uy = self.pidFunction(self.SP[-1],self.Iy[-1], self.Iy[-2], self.kPID[9], self.kPID[10],
                              self.kPID[5], self.kPID[4], self.kPID[3])
        self.Ux = np.append(self.Ux, ux)
        self.Uy = np.append(self.Uy, uy)
        servoData = Float32MultiArray()
        servoData.data = [ux, uy]
        #self.pubServo.publish(servoData)
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
    def closeNode(self): 
        #plotTwoAxis(self.IxPlot, self.IyPlot, self.timeSeries, 'Filtered Touchscreen Reading', 'Time (s)', 'Coordinate Position', 'touchscreenFiltered', 'X-Axis', 'Y-Axis')
        #plotTwoAxis(self.Ux, self.Uy, self.timeSeries, 'PID Value', 'Time (s)', 'Control Value', 'PIDControlValue', 'X-axis', 'Y-Axis', limit=True)
        saveArray(self.IxPlot, self.IyPlot, self.timeSeries, 'touchScreenFilteredData')
        saveArray(self.Ux, self.Uy, self.timeSeries, 'PIDControlValueData')

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            P = PIDClass()
            P.initNode()
            P.runNode()
            P.closeNode()
    except rospy.ROSInterruptException(): 
        pass
    

    