#!/usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University
# Fall 2023
# -*- coding: utf-8 -*-
"""
Interface for the touchscreen. This code executes a ROS node that publishes the 
coordinates of the touchscreen to the ROS topic /touchscreenData.
This code defines a ROS node that publishes the coordinates of a 
touchscreen to the ROS topic /touchscreenData. The touchScreen class initializes the 
ROS node and the necessary USB device for the touchscreen. The initTouchScreen method 
initializes the USB device and the fetchDataFromTouchScreen method fetches the data from the touchscreen. 
The publishToTopic method publishes the fetched data to the ROS topic. 
The movingAverage method calculates the moving average of the data. 
The while loop in the __main__ block fetches the data from the touchscreen and publishes it to the 
ROS topic at a specified rate
"""

# Import necessary modules here
import os, time, signal, rospy
import usb.core, usb.util
import numpy as np
from std_msgs.msg import Float32MultiArray
from constants import Constants
import time

# Class definition
class touchScreen:
    def __init__(self):
        # Parameters
        self.pubRate = 120 
        self.queueSize = 1
        c = Constants()
        self.kPID = c.PIDConstants()
        
        # Initialize ROS node
        rospy.init_node('Touchscreen', anonymous=True)
        self.t = rospy.Time.now() # Initialize time
        self.publisher = rospy.Publisher('touchscreenData', Float32MultiArray, queue_size=self.queueSize)
        # Initialize arrays 
        self.arr = np.empty(shape=(2,1))
    
    def initTouchScreen(self, idVendor = 0x04d8, idProduct = 0x0c02):
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
        self.dev = dev
        self.ep_in = ep_in
        self.ep_out = ep_out
    def fetchDataFromTouchScreen(self, dev, ep_in, ep_out):
        # Fetch data from touchscreen
        try:
            self.dev.write(ep_out.bEndpointAddress, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            data = self.dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize)
            X_Coordintate = int((data[2] * 256 + data[1] - 280)*100/3900)
            Y_Coordintate = int((data[4] * 256 + data[3] - 172)*100/3891)
            X_Coordintate = np.array(X_Coordintate)[np.newaxis]
            Y_Coordintate = np.array(Y_Coordintate)[np.newaxis]
        except usb.core.USBError as e:
            data = None 
        return [X_Coordintate, Y_Coordintate]
    
    def publishToTopic(self, data):
        # Publish data to ROS topic
        msg = Float32MultiArray()
        self.arr = np.append(self.arr, data, axis=1)
        msg.data = [float(data[0][0]), float(data[1][0])]
        # if len(self.arr[0,:]) and len(self.arr[1,:]) > self.kPID[7]:
        #     self.xyFiltered = self.movingAverage(self.arr[0,:], self.arr[1,:], self.kPID[7], self.kPID[8])
        self.publisher.publish(msg)

    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]#Ix, Iy


if __name__  == '__main__': 
    # Initialize touchscreen
    ts = touchScreen()
    ts.initTouchScreen()
    try:
        while not rospy.is_shutdown():
            # Fetch data from touchscreen
            [X_Coordintate, Y_Coordintate] = ts.fetchDataFromTouchScreen(ts.dev, ts.ep_in, ts.ep_out)
            # Publish data to ROS topic
            ts.publishToTopic([X_Coordintate, Y_Coordintate])

            time.sleep(1/ts.pubRate)
    except rospy.ROSInterruptException:
        pass
        

    

