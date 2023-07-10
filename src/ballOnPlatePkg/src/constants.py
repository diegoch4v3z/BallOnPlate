#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

class Constants: 
    def PIDConstants(self): 
        # PID 
        kP_x = 1.1
        kI_x = 0.0
        kD_x = 0.3
        kP_y = 1.1
        kI_y = 0.0 
        kD_y = 0.3
        setPoint = 0
        kernelSize = 10
        kernelDelay = -3
        iErr = 0
        dt = 0.01
        PIDconstants = [kP_x, kI_x, kD_x, kP_y, kI_y, kD_y, setPoint, kernelSize, kernelDelay, iErr, dt]
        return PIDconstants