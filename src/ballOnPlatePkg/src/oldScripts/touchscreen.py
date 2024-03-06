#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
from typing import Any 
import usb.core, usb.util
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis, saveArray, saveArray2, saveArrayLQR, saveTimeArrayTouchscreen, saveArrayACC
from constants import Constants



class touchScreen: 
    def __init__(self): 
        # Parameters 
        self.pubRate = 120 # Hertz 
        self.queueSizePar = 10
        rospy.init_node('Touchscreen', anonymous=True)
        self.start_time = rospy.Time.now() 
        self.t = rospy.Time.now()
        self.dataXPlot = np.array([])
        self.dataYPlot = np.array([])
        self.dataXPlotFiltered = np.array([])
        self.dataYPlotFiltered = np.array([])
        self.timeSeries = np.array([])
        self.frequency = np.array([])
        self.Ix = np.array([])
        self.Iy = np.array([])
        self.tn = np.array([])
        self.plot = True
        c = Constants()
        self.kPID = c.PIDConstants()

    def initNode(self, idVendor = 0x04d8, idProduct = 0x0c02):  
        print('Initializing touchscreen...')
        dev = usb.core.find(idVendor = idVendor, idProduct = idProduct)
        ep_in = dev[0].interfaces()[0].endpoints()[0]
        ep_out = dev[0].interfaces()[0].endpoints()[1]
        intf = dev[0].interfaces()[0].bInterfaceNumber
        dev.reset()
    
        if dev is None: 
            raise ValueError('Device not found')
        else: 
            print('Connected touchscreen... ', dev)

        if dev.is_kernel_driver_active(intf):
            dev.detach_kernel_driver(intf)
            usb.util.claim_interface(dev, intf)

        # Create ROS Node 
        self.pub = rospy.Publisher('touchscreenData', Float32MultiArray, queue_size=self.queueSizePar)
        self.rate = rospy.Rate(self.pubRate)
        self.dev = dev
        self.ep_in = ep_in
        self.ep_out = ep_out
    
    def getData(self, dev, ep_in, ep_out): 
        screen_x_lim_up = 1870
        screen_x_lim_down = 2200#1991
        screen_y_lim_up = 3920
        screen_y_lim_down = 1995
        try: 
            data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize) #Collected data from the touchscreen
            
            #X_coordinate = (((data[2] - 7)*255 + data[1])*-1)/(screen_x_lim_down) 
            #Y_coordinate = (((data[4] - 7)*255 + data[3])*-1)/(screen_y_lim_down) 
            rospy.loginfo("Value 2: %s, Value 2: %s", X_coordinate)

        except usb.core.USBError as e: 
            data = None
        return [X_coordinate, Y_coordinate]
    
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]#Ix, Iy

    def runNode(self):

        try:
            while not rospy.is_shutdown():
                self.tn = np.append(self.tn, (rospy.Time.now() - self.t).to_sec())
                self.current_time = (rospy.Time.now()).to_sec()
                data = Float32MultiArray()
                coordinate = self.getData(self.dev, self.ep_in, self.ep_out)
                self.dataXPlot = np.append(self.dataXPlot, coordinate[0])
                self.dataYPlot = np.append(self.dataYPlot, coordinate[1])

                # if len(self.dataXPlot) > self.kPID[7] and len(self.dataYPlot) > self.kPID[7]: 
                #     self.xyFiltered = self.movingAverage(self.dataXPlot, self.dataYPlot, self.kPID[7],self.kPID[8])
                #     self.dataXPlotFiltered = np.append(self.dataXPlotFiltered, self.xyFiltered[0])
                #     self.dataYPlotFiltered = np.append(self.dataYPlotFiltered, self.xyFiltered[1])

                data.data = [coordinate[0], coordinate[1]]
                #self.pub.publish(data)
                #rospy.loginfo("Published Touchscreen Coordinates: %s", data.data)
                self.rate.sleep()

    
                #self.timeSeries = np.append(self.timeSeries, self.current_time)
                self.deltaTime = rospy.Time.now().to_sec() - self.current_time
                self.timeSeries = np.append(self.timeSeries, self.current_time - self.start_time.to_sec())
                self.frequency = np.append(self.frequency, 1/(self.deltaTime))
                
            if self.plot: 
                #saveArrayLQR(self.dataXPlot, self.dataYPlot, self.timeSeries, 'touchScreenReadingRaw')
                #saveArrayLQR(self.dataXPlotFiltered, self.dataYPlotFiltered, self.timeSeries[:len(self.dataXPlotFiltered)], 'touchScreenReadingFiltered')
                #saveTimeArrayTouchscreen(self.timeSeries, 'touchScreenReadingTiming')
                #saveArrayACC(self.dataXPlot, self.dataYPlot, self.timeSeries, 'touchScreenReadingRaw')
                saveArray(self.dataXPlot, self.dataYPlot, self.timeSeries, 'touchScreenReadingRaw')
                saveArray(self.frequency, self.frequency, self.timeSeries, 'touchScreenReadingFrequency')
                saveArray(self.tn, self.tn, self.timeSeries, 'touchScreenReadingMeasurementTime')
                
        except rospy.ROSInterruptException: 
            pass
if __name__ == '__main__':
    t = touchScreen()
    t.initNode()
    t.runNode()