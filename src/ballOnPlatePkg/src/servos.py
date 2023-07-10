#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import os
from typing import Any
import Adafruit_PCA9685 


class servos: 
    def __init__(self): 
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=2)
        self.pwm.set_pwm_freq(60)
        self.pwm.set_pwm(0, 0, 375)
        self.pwm.set_pwm(1, 0, 375)
    def __call__(self, servoNo, pwmVal, randVal = 0):
        self.pwm.set_pwm(servoNo, randVal, pwmVal)

    def uSafety(u):
        if u > 1.1 or u < -1.1:
            u = 1
        elif u < -1.1: 
            u = -1
        return u