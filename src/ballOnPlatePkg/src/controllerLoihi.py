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
import nengo_loihi.hardware.snips as snips
from save import saveActivity
import logging
from nxsdk.graph.processes.phase_enums import Phase


# Class definition


global queueSize, pubRate, x, y, dt, sim
queueSize = 1
pubRate = 60
x = 0
y = 0
dt = 0.001



class controller:
    def __init__(self):
        # constants
        self.x = 0
        self.y = 0
        self.dt = 0.001
        
        # build Nengo Model
        self.net = self.build()
        
        # setup loihi 
        self.sim, self.board, self.inputChannel, self.feedbackChannel = self.setupLoihi(self.net)
        
        # Init ROS node
        self.pub, self.sub = self.initNode()


        
        
       
        
        
    def __call__(self, t, values):
        return self.handle_output()
    
    def callback(self, msg): 
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.sim.step()
        rospy.loginfo("x: %s, y: %s", self.x, self.y)
        self.inputChannel.write(1, [int(self.x)])
        
        
        
        
    
    def handle_output(self): 
        return [self.x, self.y]
        
    def build(self):
        net = nengo.Network(label='Controller', seed=1)
        neuronType = nengo_loihi.LoihiLIF()
        tau = 0.01 
        probeTau = 0.01 
        with net:
            # nodes
            self.XY_N = nengo.Node(self.__call__, size_in = 2, size_out = 2) 
            
            # ensembles
            self.XY_E = nengo.Ensemble(n_neurons=100, dimensions=1, neuron_type=neuronType, radius= 110, label = 'XY_E')
    

            # connections
            #XY_C = nengo.Connection(XY_N, XY_E, synapse=tau, label='XY_C')
            nengo.Connection(self.XY_N[0], self.XY_E, synapse=tau, label='XY_C')
            self.outProbe = nengo.Probe(self.XY_E, synapse=probeTau, label='outProbe')
        return net
    
    
        
    #def run(self): 
        
    def setupLoihi(self, net):
        # Set up Loihi
        os.environ.update({'KAPOHOBAY': '1'})
        nengo_loihi.set_defaults()
        sim = nengo_loihi.Simulator(net, dt=self.dt, progress_bar=False, target='loihi', hardware_options={"snip_max_spikes_per_step": 300, "n_chips": 2})
        
        
        board = sim.sims["loihi"].nxsdk_board
        hw = sim.sims["loihi"]
        hw.use_snips = False
        
        
        # Set up a SNIP to do management 
        cppFile = os.path.dirname(os.path.realpath(__file__)) + "/spikingX.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        #blocks = sim.model.objs[self.XY_E]["out"]
        #blocks = blocks if isinstance(blocks, (list, tuple)) else [blocks]
        funcName = "runSpiking"
        guardName = "doSpiking"
        embeddedProcess = board.createSnip(
            phase=Phase.EMBEDDED_MGMT,
            cFilePath=cppFile,
            includeDir=includeDir,
            funcName=funcName,
            guardName=guardName)

        # Input Channel
        inputChannel = board.createChannel(name=b'input', messageSize=4, numElements=10000) # Check if this configuration is appropriate
        inputChannel.connect(None, embeddedProcess)
        # Feedback Channel
        feedbackChannel = board.createChannel(name=b'feedback', messageSize=4, numElements=10000)
        feedbackChannel.connect(embeddedProcess, None)
        
        import time
        # starts the loihi
        sim.sims["loihi"].connect()


        
        return sim, board, inputChannel, feedbackChannel
    
    def initNode(self): 
        rospy.init_node('controller', anonymous=True)
        sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=queueSize)
        pub = rospy.Subscriber('servoData', Float32MultiArray, queue_size=queueSize)
        self.inputChannel.write(1, [0])
        return pub, sub 
    
    def shutdown(self): 
        self.sim.close()
        sim.sims["loihi"].close()
    
    def get_probes(self):
        return self.outProbe
        
        




if __name__ == '__main__':
    c = controller()
    sim = c.sim

    try:
        while not rospy.is_shutdown():
            #net, probe = network()
            #hw, sim = loihiPrep(net)
            time.sleep(1/pubRate)
            probes = c.get_probes()
            
    except rospy.ROSInterruptException:
        c.shutdown()
        pass