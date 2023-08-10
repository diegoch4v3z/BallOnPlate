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
from plots import plotTwoAxis, saveArray
from constants import Constants

class touchScreen: 
    def __init__(self): 
        rospy.init_node('Touchscreen', anonymous=True)
        self.start_time = rospy.Time.now() 
        self.dataXPlot = np.array([])
        self.dataYPlot = np.array([])
        self.timeSeries = np.array([])
        self.Ix = np.array([])
        self.Iy = np.array([])
        self.plot = True
        c = Constants()
        self.kPID = c.PIDConstants()
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]


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
        self.pub = rospy.Publisher('touchscreenData', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(120)
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
            # X_coordinate = (((data[2] - 10)*256 + data[1])*-1)/(screen_x_lim_down)
            # Y_coordinate = (((data[4] - 7)*256 + data[3])*-1)/(screen_y_lim_down)
            X_coordinate = (((data[2] - 8)*256 + data[1])*-1)/(screen_x_lim_down)
            Y_coordinate = (((data[4] - 8)*256 + data[3])*-1)/(screen_y_lim_down)
        except usb.core.USBError as e: 
            data = None
        return [X_coordinate, Y_coordinate]
    
    def runNode(self):

        try:
            while not rospy.is_shutdown():
                self.current_time = (rospy.Time.now() - self.start_time).to_sec() #(time.time() - self.start_time)#(rospy.Time.now() - self.start_time).to_sec()
                self.timeSeries = np.append(self.timeSeries, self.current_time)
                data = Float32MultiArray()
                coordinate = self.getData(self.dev, self.ep_in, self.ep_out)
                self.dataXPlot = np.append(self.dataXPlot, coordinate[0])
                self.dataYPlot = np.append(self.dataYPlot, coordinate[1])
                data.data = [coordinate[0], coordinate[1]]
                self.pub.publish(data)
                self.rate.sleep()
            if self.plot: 
                saveArray(self.dataXPlot, self.dataYPlot, self.timeSeries, 'touchScreenReadingRaw')#, 'TouchScreen Reading', 'Time (s)', 'Coordinate Position', 'touchScreenData', 'X-Axis', 'Y-Axis', limit=True) 
        except rospy.ROSInterruptException: 
            pass
if __name__ == '__main__':
    t = touchScreen()
    t.initNode()
    t.runNode()