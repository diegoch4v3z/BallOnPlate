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
from plots import plotTwoAxis

class touchScreen: 
    def __init__(self): 
        self.dataXPlot = np.array([0, 0])
        self.dataYPlot = np.array([0, 0])
        self.timeSeries = np.array([0, 0])
        self.plot = True

    def initNode(self, idVendor = 0x04d8, idProduct = 0x0c02):
        rospy.init_node('Touchscreen', anonymous=True)
        self.start_time = rospy.Time.now()
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
        screen_x_lim_down = 1917
        screen_y_lim_up = 3920
        screen_y_lim_down = 1666
        try: 
            data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize) #Collected data from the touchscreen
            X_coordinate = (((data[2] - 8)*256 + data[1])*-1)/(screen_x_lim_down)
            Y_coordinate = (((data[4] - 8)*256 + data[3])*-1)/(screen_y_lim_down)
        except usb.core.USBError as e: 
            data = None
        return [X_coordinate, Y_coordinate]
    
    def runNode(self):
        try:
            while not rospy.is_shutdown():
                self.current_time = (rospy.Time.now() - self.start_time).to_nsec()
                self.timeSeries = np.append(self.timeSeries, self.current_time)
                data = Float32MultiArray()
                coordinate = self.getData(self.dev, self.ep_in, self.ep_out)
                self.dataXPlot = np.append(self.dataXPlot, coordinate[0])
                self.dataYPlot = np.append(self.dataYPlot, coordinate[1])
                data.data = [coordinate[0], coordinate[1]]
                self.pub.publish(data)
                self.rate.sleep()
            if self.plot: 
                plotTwoAxis(self.dataXPlot, self.dataYPlot, self.timeSeries, 'TouchScreen Reading', 'Time (ns)', 'Coordinate Position', 'touchscreen') 
        except rospy.ROSInterruptException: 
            pass
if __name__ == '__main__':
    t = touchScreen()
    t.initNode()
    t.runNode()