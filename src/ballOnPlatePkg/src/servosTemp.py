#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os
from typing import Any
from std_msgs.msg import Float32MultiArray
import Adafruit_PCA9685
import rospy, time, datetime
import numpy as np
from plots import plotTwoAxis, saveArray, saveArrayLQR, saveTimeArrayServos, saveArrayACC
from scipy import signal

class servos:
    def __init__(self):
        #rospy.init_node('Servo', anonymous=True)
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=2)
        self.pwm.set_pwm_freq(60)
        self.pwm.set_pwm(0, 0, 378)
        self.pwm.set_pwm(1, 0, 285)
        # Around Y - For the X axis. 
        # 
        # 245 is the minimum value for the servo inclination negative X
        # 508 is the maximum value for the servo inclination positive X
        # 370 is the middle value for the servo
        
        
        
        
        # numerator = [0, -0.00020089]  
        # denominator = [1, -1.8566, 0.8651]
        # transfer_function = signal.TransferFunction(numerator, denominator, dt=0.1)
        # t_out, y_out, _ = signal.dlsim(transfer_function, u, t)

        # Around X
        # 260 is the minimum value for the servo
        # 520 is the maximum value for the servo
        # 375 is the middle value for the servo

        

        # angles_Y = np.linspace(260, 520, 30)
        # measurments = []
        # for i in range(len(angles_Y)):
        #     self.pwm.set_pwm(1, 0, int(angles_Y[i]))
        #     measurments.append(float(input("Enter the measurment: ")))
        #     time.sleep(0.1)
        #     np.savetxt('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/servoY.txt', measurments)
        #     print(measurments)





if __name__ == '__main__':
    s = servos()
    