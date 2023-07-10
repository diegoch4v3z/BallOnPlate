#! /usr/bin/python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os, time, rospy
import matplotlib.pyplot as plt
from touchscreen import touchScreen
from servos import servos
from PID import PIDClass


rospy.init_node('main', anonymous=True)


if __name__ == "__main__": 
    start_time = time.time()
    rate = rospy.Rate(50)
    p = PIDClass()
    try: 
        while not rospy.is_shutdown(): 
            rate.sleep()
            if time.time() - start_time >= 10:
                break
    except rospy.ROSInterruptException: 
        pass
        