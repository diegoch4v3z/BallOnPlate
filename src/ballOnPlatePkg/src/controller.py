#!/usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University
# Fall 2023
# -*- coding: utf-8 -*-



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
        # constants
        c = Constants()
        self.kPIDNengo = c.PIDNengoConstants() 
        self.kPID = c.PIDConstants()
        self.dt = 0.01
        self.filter = False
        self.probe = True
        self.queueSize = 1
        self.runType = 0 
        self.pubRate = 200

        # arrays 
        self.xArray = np.array([])
        self.yArray = np.array([])
        
        # Nengo Model
        self.build()
        self.sim = nengo_loihi.Simulator(self.model, dt=self.dt, progress_bar=False, target='sim')

        # ROS publishers and subscribers
        rospy.init_node('controller', anonymous=True)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=self.queueSize)
        self.pub = rospy.Publisher('servoData', Float32MultiArray, queue_size=self.queueSize)

    def __call__(self, t, values): 
        servoData = Float32MultiArray()
        servoData.data = [values[0], values[1]]
        #self.pub.publish(servoData)
        rospy.loginfo("Obtained valued from node: %s", values)
        return self.handle_output()
    
    def callback(self, msg):
        # this one is called every time a new message is received
        self.start_time = time.time()
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.xArray = np.append(self.xArray, self.x)
        self.yArray = np.append(self.yArray, self.y)
        rospy.loginfo("x: %s, y: %s", self.x, self.y)
        self.sim.step()

        
    
    def handle_output(self):
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
        delayX = DelayX(1, timesteps=int(0.05/0.005))
        delayY = DelayY(1, timesteps=int(0.05/0.005))
        with self.model:
            # N - nodes, E - ensembles, C - connections, P - probes
            posXY_N = nengo.Node(self.__call__, size_out=2, size_in=2, label='posXY_node')
            posXY_E = nengo.Ensemble(n_neurons=400, dimensions=2, neuron_type=neuronType, radius=100, label='posXY_E', max)
            
            # connections
            nengo.Connection(posXY_N, posXY_E, synapse=0.001, label='XY_C')
            nengo.Connection(posXY_E, posXY_N, synapse=0.5, label='XY_C')


    def run(self):
        try: 
            while not rospy.is_shutdown(): 
                time.sleep(1/self.pubRate)
        except rospy.ROSInterruptException: 
            self.sim.close()
            pass 
    def saveProbe(self):
        if self.probe: 
            rospy.loginfo('Saving probe data...')
        else: 
            pass


if __name__ == '__main__':
    rospy.loginfo('Starting controller node...')
    c = controller()
    rospy.loginfo('Running controller node...')
    c.run()
    c.saveProbe()