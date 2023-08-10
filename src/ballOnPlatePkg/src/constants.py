#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

class Constants: 
    def PIDConstants(self): 
        # PID 
        kP_x = 2.5
        kI_x = 0.0
        kD_x = 0.9
        kP_y = 2.5
        kI_y = 0.0 
        kD_y = 0.9
        setPoint = 0
        kernelSize = 10
        kernelDelay = -3
        iErr = 0
        dt = 0.01 #0.01
        PIDconstants = [kP_x, kI_x, kD_x, kP_y, kI_y, kD_y, setPoint, kernelSize, kernelDelay, iErr, dt]
        return PIDconstants
    def PIDNengoConstants(self): 
        kP_x = 2.5     #2.5 
        kI_x = 0.0
        kD_x = -1.5#-1.5#-1.2     #-0.7   
        kP_y = 2.5      #2.5 
        kI_y = 0.0 
        kD_y = -1.5#-1.2     #-0.7
        setPoint = 0
        dt = 0.001
        PIDconstants = [kP_x, kI_x, kD_x, kP_y, kI_y, kD_y, setPoint, dt]
        return PIDconstants