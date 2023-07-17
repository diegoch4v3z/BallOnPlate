#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import nengo, rospy, sys
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis, saveArray
import numpy as np
import scipy

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
            posXY_node = nengo.Node(self.PIDNengoNode, size_out=2, size_in=2)
            setpoint = nengo.Node([0, 0])

            #Ensembles
            posXY_ensemble = nengo.Ensemble(n_neurons=200,dimensions=2, neuron_type=nengo.Direct())
            setPointEnsemble = nengo.Ensemble(n_neurons=200, dimensions=2)
            error = nengo.Ensemble(n_neurons=400, dimensions=2)

            d_e_x = nengo.Ensemble(n_neurons=300, dimensions = 1)
            d_e_y = nengo.Ensemble(n_neurons=300, dimensions = 1)

            u = nengo.Ensemble(n_neurons=200, dimensions=2)


            # Conections 
            nengo.Connection(setpoint, setPointEnsemble, synapse=None)
            nengo.Connection(posXY_node, posXY_ensemble, synapse=0.001)
            nengo.Connection(setPointEnsemble, error, synapse=None)
            nengo.Connection(posXY_ensemble, error, transform=-1, synapse=None)

            nengo.Connection(error[0], d_e_x, synapse=0.005,transform=1000)#1000)
            nengo.Connection(error[0], d_e_x, synapse=0.01, transform=-1000)#-1000)
            nengo.Connection(error[1], d_e_y, synapse=0.005,transform=1000)#1000)
            nengo.Connection(error[1], d_e_y, synapse=0.01, transform=-1000)#-1000)
            def control(x):
                return [x[0]*self.kPID[0],x[1]*self.kPID[3]]   
            nengo.Connection(error[0], u[0], transform=self.kPID[0]) 
            nengo.Connection(error[1], u[1], transform=self.kPID[3]) 
            nengo.Connection(d_e_x, u[0], transform=self.kPID[2])
            nengo.Connection(d_e_y, u[1], transform=self.kPID[5])

            
            nengo.Connection(u, posXY_node, synapse=0.1)
            if probe: 
                self.probesNengoPosXYEnsemble = nengo.Probe(posXY_ensemble, synapse=0.1)
                self.probesSetPointEnsemble = nengo.Probe(setPointEnsemble, synapse=0.1)
                self.probesError = nengo.Probe(error, synapse=0.1)
                self.probesG = nengo.Probe(u, synapse=0.1)
        
    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    def get_model(self): 
        return self.model
    def get_probes(self): 
        return [self.probesNengoPosXYEnsemble, self.probesSetPointEnsemble, 
                self.probesError, self.probesG]

class touchScreenCoordinates: 
    def __init__(self): 
        self.x = 0
        self.y = 0
        rospy.init_node('PIDNengo', anonymous=True)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(500)
    def callback(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
    def __call__(self, t, values): 
        servoData = Float32MultiArray()
        servoData.data = [values[0], values[1]]
        self.pubServo.publish(servoData)
        self.rate.sleep()
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
        sim = nengo.Simulator(model, dt=0.01)
        probes = p.get_probes()
        self.start_time = rospy.Time.now()
        try:
            for i in range(1000): 
                self.current_time = (rospy.Time.now()).to_sec()#(rospy.Time.now() - self.start_time).to_sec()
                self.timeSeries = np.append(self.timeSeries, self.current_time)
                sim.step()
                dataProbe0 = sim.data[probes[0]]
                #dataProbe1 = sim.data[probes[1]]
                dataProbe2 = sim.data[probes[2]]
                dataProbe3 = sim.data[probes[3]]
        except SystemExit:
            pass
        saveArray(dataProbe0[:, 0], dataProbe0[:, 1], self.timeSeries, 'touchScreenNengoData')
        saveArray(dataProbe2[:, 0], dataProbe2[:, 1], self.timeSeries, 'ErrorNengoData')
        saveArray(dataProbe3[:, 0], dataProbe3[:, 1], self.timeSeries, 'ControlSignalNengoData')
        #plotTwoAxis(dataProbe0[:, 0], dataProbe0[:, 1], self.timeSeries, 'TouchScreen Reading', 'Time (s)', 'Coordinate Position', 'touchscreenNengo', 'X-Axis', 'Y-Axis') 
        #plotTwoAxis(dataProbe1[:, 0], dataProbe1[:, 1], self.timeSeries, 'SetPoint', 'Time (s)', 'Coordinate Position', 'setPointNengo')
        #plotTwoAxis(dataProbe2[:, 0], dataProbe2[:, 1], self.timeSeries, 'Error', 'Time (s)', 'Error', 'ErrorNengo')
        #plotTwoAxis(dataProbe3[:, 0], dataProbe3[:, 1], self.timeSeries, 'Control Signal', 'Time (s)', 'Error', 'ControlSignalNengo')
        sys.exit()
def send_interrupt_to_other_code(self):
    # Assuming the other code's process name is "other_code.py"
    import os
    process_name = "PID.py"
    process_name1 = "touchscreen.py"
    os.system("pkill -SIGINT -f {}".format(process_name))
    os.system("pkill -SIGINT -f {}".format(process_name1))

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            runModel()
        send_interrupt_to_other_code()
    except rospy.ROSInterruptException: 
        pass