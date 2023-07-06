#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal
from typing import Any 
import usb.core, usb.util
import numpy, rospy
from std_msgs.msg import Int32MultiArray

class touchScreen: 
    def __init__(self, idVendor = 0x04d8, idProduct = 0x0c02):
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
        rospy.init_node('Touchscreen')
        self.pub = rospy.Publisher('Touchscreen data', Int32MultiArray, queue_size=10)
        self.rate = rospy.Rate(50)
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
    
    def __call__(self, *args: Any, **kwds: Any) -> Any:
        data = Int32MultiArray()
        coordinate = self.getData(self.dev, self.ep_in, self.ep_out)
        data.data = [coordinate[0], coordinate[1]]

        self.rate.sleep()
