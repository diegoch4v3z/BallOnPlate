#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import nengo, rospy
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis
import numpy as np

class PIDNengo: 
    def __init__(self): 
        # Import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        self.i = 0
        self.buildNengoModel(probe = True)
    def buildNengoModel(self, probe = True): 
        self.model = nengo.Network(label='PID', seed=1)
        with self.model:
            self.PIDNengoNode = touchScreenCoordinates()
            posXY_node = nengo.Node(self.PIDNengoNode, size_out=2)
            posXY_ensemble = nengo.Ensemble(n_neurons=200,dimensions=2, neuron_type=nengo.Direct())
            posXY_node_ensemble_connection = nengo.Connection(posXY_node, posXY_ensemble, synapse=None)
            if probe: 
                self.probesNengoPosXYEnsemble = nengo.Probe(posXY_ensemble, synapse=0.1)

    def initNode(self): 
        self.sim = nengo.Simulator(self.model, dt=self.kPID[7])

    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    def get_model(self): 
        return self.model
    def get_probes(self): 
        return self.probesNengoPosXYEnsemble

class touchScreenCoordinates: 
    def __init__(self): 
        self.x = 0
        self.y = 0
        rospy.init_node('PIDNengo', anonymous=True)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(80)
    def callback(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
    def __call__(self, values): 
        # self.count += 1
        # if self.count == 17:
        #     self.count = 0
        #     #self.references = self.reference()
        #     #print("references llamadas en el call", self.references)
        #     self.handle_input(values)
        return self.handle_output()
    def handle_output(self): 
        return [self.x, self.y]

class runModel: 
    def __init__(self):
        self.timeSeries = np.array([])
        p = PIDNengo()
        model = p.get_model()
        sim = nengo.Simulator(model, 0.001)
        probes = p.get_probes()
        self.start_time = rospy.Time.now()

        for i in range(3000): 
            self.current_time = (rospy.Time.now() - self.start_time).to_nsec()
            self.timeSeries = np.append(self.timeSeries, self.current_time)
            sim.step()
            dataProbe = sim.data[probes]
        print(len(dataProbe[:, 0]), len(self.timeSeries))
        plotTwoAxis(dataProbe[:, 0], dataProbe[:, 1], self.timeSeries, 'TouchScreen Reading', 'Time (ns)', 'Coordinate Position', 'touchscreenNengo') 
        





if __name__ == '__main__':
    try: 
        runModel()
    except rospy.ROSInterruptException: 
        pass