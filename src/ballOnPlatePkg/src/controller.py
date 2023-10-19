#!/usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University
# Fall 2023
# -*- coding: utf-8 -*-
"""



"""

# import modules
import os, time, signal, rospy, sys, nengo, scipy, datetime, message_filters, nengo_loihi
import matplotlib.pyplot as plt
from constants import Constants
from std_msgs.msg import Float32MultiArray
import numpy as np
import nengo_loihi.hardware as hardware
from save import saveActivity

# Class definition

class DelayX: 
    def __init__(self, dimensions, timesteps=50):
        self.history = np.zeros((timesteps, dimensions))

    def step(self, t, x):
        self.history = np.roll(self.history, -1)
        self.history[-1] = x
        return self.history[0]

class DelayY: 
    def __init__(self, dimensions, timesteps=50):
        self.history = np.zeros((timesteps, dimensions))

    def step(self, t, x):
        self.history = np.roll(self.history, -1)
        self.history[-1] = x
        return self.history[0]

class controller: 
    def __init__(self): 
        # import constants
        c = Constants()
        self.kPIDNengo = c.PIDNengoConstants() 
        self.kPID = c.PIDConstants()
        self.dt = 0.008
        self.filter = True
        self.probe = True
        self.queueSize = 1
        self.runType = 0 # 0 - Simulation, 1 - Emulation, 2 - Hardware
        self.pubRate = 120
        # build Nengo model
        self.build()
        self.sim = nengo.Simulator(self.model, dt=self.dt, progress_bar=False, optimize=True)

        # ROS publishers and subscribers
        rospy.init_node('controller', anonymous=True)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=self.queueSize)
        self.pub = rospy.Publisher('servoData', Float32MultiArray, queue_size=self.queueSize)

        # data storage arrays
        self.x = 0 
        self.y = 0 
        self.xArray = np.array([])
        self.yArray = np.array([])
        self.nengo_process_time = np.array([])
    def __call__(self, t, values): 
        # this one can control the rate of processing 
        servoData = Float32MultiArray()
        servoData.data = [values[0], values[1]]
        #self.pub.publish(servoData)
        self.end_time = time.time()
        self.nengo_process_time = np.append(self.nengo_process_time, self.end_time - self.start_time)
        return self.handle_output()
    
    def callback(self, msg):
        # this one is called every time a new message is received
        self.start_time = time.time()
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.xArray = np.append(self.xArray, self.x)
        self.yArray = np.append(self.yArray, self.y)
        rospy.loginfo('x: %f, y: %f', self.x, self.y)
        self.sim.step()
        
        
    
    def handle_output(self):
        self.xArray = np.append(self.xArray, self.x)
        self.yArray = np.append(self.yArray, self.y)
        if self.filter == True: 
            if len(self.xArray) > self.kPID[7] and len(self.yArray) > self.kPID[7]:
                self.xyFiltered = self.movingAverage(self.xArray, self.yArray, self.kPID[7], self.kPID[8])
                self.xArray[-1] = self.xyFiltered[0]
                self.yArray[-1] = self.xyFiltered[1]    
        else: 
            pass 
        return [self.xArray[-1], self.yArray[-1]]

    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]
    
    def build(self): 
        self.model = nengo.Network(label='Controller', seed=1)
        neuronType = nengo_loihi.LoihiLIF()
        tau = 0.02
        probeTau = 0.1
        delayX = DelayX(1, timesteps=int(0.05/0.005))
        delayY = DelayY(1, timesteps=int(0.05/0.005))
        with self.model:
            # N - nodes, E - ensembles, C - connections, P - probes
            
            # nodes
            XY_N = nengo.Node(self.__call__, size_in=2, size_out=2) 


            # ensembles
            XY_E = nengo.Ensemble(n_neurons=1000, dimensions=2, neuron_type=neuronType, radius= 4000, label = 'XY_E')

            # connections
            XY_C = nengo.Connection(XY_N, XY_E, synapse=None, label='XY_C')

            # probe 
            if self.probe: 
                rospy.loginfo('Probing data...')
                self.XY_E_P = nengo.Probe(XY_E, synapse=probeTau, label='probeXY')
                self.XY_C_P = nengo.Probe(XY_C, synapse=probeTau, label='probeXY_C')
    def run(self):
        try: 
            while not rospy.is_shutdown(): 
                time.sleep(1/self.pubRate)
        except rospy.ROSInterruptException: 
            pass 
    def saveProbe(self):
        if self.probe: 
            rospy.loginfo('Saving probe data...')
            saveActivity(self.sim.data[self.XY_E_P][:, 0], self.sim.data[self.XY_E_P][:, 1], self.nengo_process_time, 'XY_E')
        else: 
            pass


if __name__ == '__main__':
    rospy.loginfo('Starting controller node...')
    c = controller()
    try:
        rospy.loginfo('Running controller node...')
        c.run()
        c.saveProbe()
    except rospy.ROSInterruptException:
        pass