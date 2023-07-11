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
    start_time = rospy.Time.now()
    rate = rospy.Rate(50)
    duration = 10
    #p = PIDClass()
    try: 
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException: 
        pass
        