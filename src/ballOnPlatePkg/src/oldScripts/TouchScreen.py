#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, signal, rospy
from typing import Any
import usb.core, usb.util
import numpy as np
from std_msgs.msg import Float32MultiArray
from constants import Constants

class touchScreen:


    def __init__(self):
        self.queueSize = 1
        rospy.init_node('Touchscreen', anonymous=True)

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
        self.pub = rospy.Publisher('touchscreenData', Float32MultiArray, queue_size=self.queueSize)
        self.rate = rospy.Rate(60)
        self.dev = dev
        self.ep_in = ep_in
        self.ep_out = ep_out
        
    def getData(self, dev, ep_in, ep_out): 
        # Get data from touchscreen
        dev.write(ep_out, [0x01, 0x00, 0x00])
        try:
            data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize, ep_in.wMaxPacketSize)
            X_Coordintate = int((data[2] * 256 + data[1] - 280)*100/3900)
            Y_Coordintate = int((data[4] * 256 + data[3] - 172)*100/3891)
        except usb.core.USBError as e: 
            data = None
        return [X_Coordintate, Y_Coordintate]
    
    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                data = self.getData(self.dev, self.ep_in, self.ep_out)
                msg = Float32MultiArray()
                msg.data = data
                self.pub.publish(msg)
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass


if __name__ == '__main__':
    t = touchScreen()
    t.initNode()
    t.runNode()