#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import nengo, rospy
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray

class PIDNengo: 
    def __init__(self): 
        # Import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        self.rate = rospy.Rate(240)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
    def buildNengoModel(self): 
        with nengo.Network(label='ballAndPlatePID') as model: 


    def initNode(self): 
        rospy.init_node('PIDNengo', anonymous=True)
        start_time = rospy.Time.now()

        sim = nengo.Simulator()



    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 

    def callback(self, msg): 
        self.msg = msg




if __name__ == '__main__':
    P = PIDNengo()
    P.initNode()
    P.runNode()
    