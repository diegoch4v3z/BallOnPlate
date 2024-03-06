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
        
        self.pub = rospy.Publisher('touchscreenData', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(120)
        self.dev = dev
        self.ep_in = ep_in 
        self.ep_out = ep_out 
    def getData(self, dev, ep_in, ep_out): 
        try: 
            data = dev.read(ep_in.bEndpointAddress, ep_in.wMaxPacketSize)
            return [(data[2]*255 + data[1]-293)/3925 - 0.5, (data[4]*255 + data[3] - 194)/3935 - 0.5]
        except usb.core.USBError as e: 
            if e.args == ('Operation timed out',): 
                return None

    def runNode(self):
        try: 
            while not rospy.is_shutdown():
                coordinate = self.getData(self.dev, self.ep_in, self.ep_out)
                data = Float32MultiArray()
                data.data = coordinate
                self.pub.publish(data)
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass
    
if __name__ == '__main__':
    t = touchScreen()
    t.initNode()
    t.runNode()


