#!/usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University
# Fall 2023
# -*- coding: utf-8 -*-
import os, time, signal, rospy, sys, nengo, scipy, datetime, message_filters, nengo_loihi
import matplotlib.pyplot as plt
from constants import Constants
from std_msgs.msg import Float32MultiArray
import numpy as np
import nengo_loihi.hardware as hardware
import nengo_loihi.hardware.snips as snips
from save import saveActivity
import logging
import matplotlib.pyplot as plt
from nxsdk.graph.processes.phase_enums import Phase

# global variables 
queueSize = 1
pubRate = 60 
x = 0
y = 0 

# Code for simple reading input and output of the coordinates of the ball 
# The code saves an image with the results of the simulation



class controller: 
    def __init__(self) -> None:
        self.x = 0 
        self.y = 0
        self.dt = 0.005
        self.timestep = 0
        self.endOfTimestep = 300
        self.probeSynapse = 0.1

        # Build the model
        self.model = self.build()
        # Set up the Loihi Simulator
        self.sim, self.board = self.setupLoihi(self.model)
        # Init ROS node
        self.pub, self.sub = self.initNode()

    def build(self): 
        # build model
        model = nengo.Network(label='Controller', seed=1)
        with model:
            # Nodes
            self.input = nengo.Node(self.__call__, size_in=2, size_out=2)
            # Intermediate Ensemble 
            self.intermediateX = nengo.Ensemble(300, 1, neuron_type=nengo_loihi.LoihiLIF(), radius=100)
            # connection
            #nengo.Connection(self.input[0], self.intermediate[0], synapse=0.001)
            nengo.Connection(self.input[0], self.intermediateX, synapse=0.001)
            #nengo.Connection(self.intermediate, self.output)
            self.probeX = nengo.Probe(self.intermediateX, synapse=self.probeSynapse)
            #self.probeY = nengo.Probe(self.intermediate[1], synapse=self.probeSynapse)
        return model
    
    def setupLoihi(self, model): 
        os.environ.update({'KAPOHOBAY': '1'})
        nengo_loihi.set_defaults()
        sim = nengo_loihi.Simulator(model, dt=self.dt, progress_bar=False, target='loihi', hardware_options={"snip_max_spikes_per_step": 300, "n_chips": 2})
        board = sim.sims["loihi"].nxsdk_board
        hw = sim.sims["loihi"]
    
        sim.sims["loihi"].connect()
        return sim, board
    def __call__(self, t, values):
        return self.handle_output()
    
    def handle_output(self):
        return [self.x, self.y]
    
    def initNode(self):
        rospy.init_node('controller', anonymous=True)
        sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=queueSize)
        pub = rospy.Subscriber('servoData', Float32MultiArray, queue_size=queueSize)
        #self.inputChannel.write(1, [0])
        return pub, sub 
    
    def callback(self, msg):
        
        if self.timestep < self.endOfTimestep:
            self.x = msg.data[0]
            self.y = msg.data[1]
            self.sim.step()
            rospy.loginfo("x: %s, y: %s", self.x, self.y)
            self.timestep = self.timestep + 1
        else:
            self.shutdown()
            rospy.signal_shutdown("End of timestep reached")


    def shutdown(self):
        self.sim.close()
        self.sim.sims["loihi"].close()

    def get_probes(self): 
        return self.probeX#, self.probeY


    

if __name__ == '__main__':
    c = controller()
    sim = c.sim
    try:
        while not rospy.is_shutdown():
            #net, probe = network()
            #hw, sim = loihiPrep(net)
            time.sleep(1/pubRate)
            probesX = c.get_probes()

        
        t_values = sim.trange()  # Provides a numpy array with all of the timesteps in the probed data

        probed_values_X = sim.data[probesX]
        #probes_values_Y = sim.data[probesY]
        # Create a plot
        plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
        plt.rcParams['font.serif'] = 'DejaVu Serif'
        plt.rcParams['pdf.fonttype'] = 42

        fig, (ax1, ax2) = plt.subplots(2, 1)
        ax1.scatter(t_values, probed_values_X, color='red', label='X-axis')
        ax1.set_ylabel('Neuron Index Cooridnate X-axis', color='black', fontsize=6)
        ax1.set_xlabel('Time (s)', color='black', fontsize=14)
        ax1.legend()
        ax1.grid()
        
        
        ax2.scatter(t_values, probed_values_X, color='red', label='Y-axis')
        ax2.set_ylabel('Neuron Index Cooridnate Y-axis', color='black', fontsize=6)
        ax2.set_xlabel('Time (s)', color='black', fontsize=14)
        ax2.legend()
        ax1.grid()

        plt.subplots_adjust(wspace=0, hspace=0)
        plt.tight_layout()
    



        plt.savefig('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/' + 'plot.png', dpi=300)
    except rospy.ROSInterruptException:
        c.shutdown()
        pass

