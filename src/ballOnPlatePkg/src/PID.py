#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
from typing import Any 
import numpy as np
import rospy, datetime
from std_msgs.msg import Float32MultiArray
from constants import Constants
from plots import plotTwoAxis, saveArray, saveArrayACC

class PIDClass: 
    def __init__(self):
        self.x = 0
        self.y = 0

        ## Import Contansts
        c = Constants()
        self.kPID = c.PIDConstants()

        ## 
        delayDt = 2
        self.Ix = np.zeros(delayDt)
        self.Iy = np.zeros(delayDt)
        self.SPx= np.zeros(delayDt)
        self.SPy = np.zeros(delayDt)
        self.Ux = np.zeros(delayDt)
        self.Uy = np.zeros(delayDt)
        self.IxPlot = np.zeros(delayDt)
        self.IyPlot = np.zeros(delayDt)
        self.xPlot = np.zeros(delayDt)
        self.yPlot = np.zeros(delayDt)
        self.timeSeries = np.zeros(delayDt)
        self.dErrxPlot = np.zeros(delayDt)
        self.dErryPlot = np.zeros(delayDt)
        self.pErrxPlot = np.zeros(delayDt)
        self.pErryPlot = np.zeros(delayDt)


    def initNode(self): 
        rospy.init_node('PID', anonymous=True)
        self.rate = rospy.Rate(240)
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
        self.current_time = (rospy.Time.now() - self.start_time).to_sec() #(time.time() - self.start_time) #
        self.timeSeries = np.append(self.timeSeries, self.current_time)

        self.x = msg.data[0]                                            # Obtain data X from the touchscreen rospy
        self.y = msg.data[1]                                            # Obtain data Y from the touchscreen rospy
        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        self.xPlot = np.append(self.xPlot, self.x)
        self.yPlot = np.append(self.yPlot, self.y)

       
        self.SPx = np.append(self.SPx, 0)                    # Add set point continously
        self.SPy = np.append(self.SPy, 0) 
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]:
            #self.Ix, self.Iy = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
        ## Add filtered values to plot 
        self.IxPlot = np.append(self.IxPlot, self.Ix[-1])
        self.IyPlot = np.append(self.IyPlot, self.Iy[-1])
        #print(self.Ix[-1], self.Iy[-1])
        
        # ux, dErrx, pErrx = self.pidFunction(self.SP[-1],self.Ix[-1], self.Ix[-2], self.kPID[9], self.kPID[10],
        #                       self.kPID[2], self.kPID[1], self.kPID[0])
        # uy, dErry, pErry = self.pidFunction(self.SP[-1],self.Iy[-1], self.Iy[-2], self.kPID[9], self.kPID[10],
        #                       self.kPID[5], self.kPID[4], self.kPID[3])
        
        ux, dErrx, pErrx = self.pidFunction(self.SPx[-1],self.Ix[-1], self.Ix[-2], self.kPID[9], self.kPID[10],
                              self.kPID[2], self.kPID[1], self.kPID[0])
        uy, dErry, pErry = self.pidFunction(self.SPy[-1],self.Iy[-1], self.Iy[-2], self.kPID[9], self.kPID[10],
                              self.kPID[5], self.kPID[4], self.kPID[3])
        self.dErrxPlot = np.append(self.dErrxPlot, dErrx)
        self.dErryPlot = np.append(self.dErryPlot, dErry)
        self.pErrxPlot = np.append(self.pErrxPlot, pErrx)
        self.pErryPlot = np.append(self.pErryPlot, pErry)
        self.Ux = np.append(self.Ux, ux)
        self.Uy = np.append(self.Uy, uy)

        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]:
            self.xyFiltered = self.movingAverage(self.Ux, self.Uy, self.kPID[7], -1)
            self.Ux[-1] = self.xyFiltered[0]
            self.Uy[-1] = self.xyFiltered[1]

        servoData = Float32MultiArray()
        servoData.data = [ux, uy]
        self.pubServo.publish(servoData)

        self.rate.sleep()
        
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        # Ix[-1] = np.sum(Ix[-kernelSize:])/kernelSize
        # Iy[-1] = np.sum(Iy[-kernelSize:])/kernelSize
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y] #Ix, Iy 
    def pidFunction(self, sp, cv, pv, iErr, dt, kD, kI, kP): 
        # K values 
        error = sp-pv 
        iErr = iErr + kI*error*dt
        dErr = (cv - pv)/dt 
        u = kP*error + kI*iErr - kD*dErr
        return u, dErr, error
    def closeNode(self):
        saveArrayACC(self.SPx, self.SPy, self.timeSeries, 'setPoint_PID')
        saveArrayACC(self.IxPlot, self.IyPlot, self.timeSeries, 'touchScreen_PID')
        saveArrayACC(self.Ux, self.Uy, self.timeSeries, 'control_PID')
        saveArrayACC(self.pErrxPlot, self.pErryPlot, self.timeSeries, 'error_PID')

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            P = PIDClass()
            P.initNode()
            P.runNode()
            P.closeNode()
    except rospy.ROSInterruptException(): 
        pass
    

    