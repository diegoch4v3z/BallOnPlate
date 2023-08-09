#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import nengo, rospy, sys
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis, saveArray, saveArray1
import numpy as np
import scipy, datetime, time
import message_filters

pop = True
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
class PIDNengo: 
    def __init__(self): 
        # Import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        self.i = 0
        self.buildNengoModel(probe = True)
    def buildNengoModel(self, probe = False): 
        self.model = nengo.Network(label='PID', seed=1)

        dt = 0.005
        delayX = DelayX(1, timesteps=int(0.05 / 0.005))
        delayY = DelayY(1, timesteps=int(0.05 / 0.005))
        with self.model:
            # Nodes
            self.PIDNengoNode = touchScreenCoordinates()
            posXY_node = nengo.Node(self.PIDNengoNode, size_out=2, size_in=2)
            setpoint = nengo.Node([0, 0])
        

            delayNodeX = nengo.Node(delayX.step, size_in=1, size_out=1)
            delayNodeY = nengo.Node(delayY.step, size_in=1, size_out=1)

            #Ensembles
            posXY_ensemble = nengo.Ensemble(n_neurons=1, dimensions=2, neuron_type=nengo.Direct(), radius=2)
            setPointEnsemble = nengo.Ensemble(n_neurons=400, dimensions=2, radius =0.001)
            errorX = nengo.Ensemble(n_neurons=400, dimensions=1, radius=2)
            errorY = nengo.Ensemble(n_neurons=400, dimensions=1, radius=2)

            d_e_x = nengo.Ensemble(n_neurons=800, dimensions = 1, radius=1)
            d_e_y = nengo.Ensemble(n_neurons=800, dimensions = 1, radius=1)

            u = nengo.Ensemble(n_neurons=1, dimensions=2, neuron_type=nengo.Direct(), radius=3)

            
            

            # Conections 
            nengo.Connection(setpoint, setPointEnsemble, synapse=None)
            nengo.Connection(posXY_node, posXY_ensemble, synapse=None)
            nengo.Connection(setPointEnsemble[0], errorX, synapse=None, transform=1.0)#synapse=nengo.Alpha(tau=0.01), transform=3)
            nengo.Connection(setPointEnsemble[1], errorY, synapse=None, transform=1.0)
            nengo.Connection(posXY_ensemble[0], errorX, synapse=None, transform=-1.0) #synapse=nengo.Alpha(tau=0.01), transform=-3)
            nengo.Connection(posXY_ensemble[1], errorY, synapse=None, transform=-1.0)

            # nengo.Connection(error[0], d_e_x, transform = 100, synapse = 0.005)#1000)
            # nengo.Connection(error[0], d_e_x, transform = -100, synapse= 0.015)#-1000)
            # nengo.Connection(error[1], d_e_y, transform= 100, synapse= 0.005)#1000)
            # nengo.Connection(error[1], d_e_y, transform=-100, synapse= 0.015)#-1000)
            # Derivative in X
            nengo.Connection(errorX, delayNodeX)
            nengo.Connection(delayNodeX, d_e_x, transform=20)
            nengo.Connection(errorX, d_e_x, transform=-20)
            # Derivative in Y 
            nengo.Connection(errorY, delayNodeY)
            nengo.Connection(delayNodeY, d_e_y, transform=20)
            nengo.Connection(errorY, d_e_y, transform=-20)
            nengo.Connection(errorX, u[0], transform=self.kPID[0]) 
            nengo.Connection(errorY, u[1], transform=self.kPID[3]) 
            nengo.Connection(d_e_x, u[0], transform=self.kPID[2])
            nengo.Connection(d_e_y, u[1], transform=self.kPID[5])

            
            nengo.Connection(u, posXY_node, synapse=0.0001)
            if probe: 
                self.probesPosXYEnsemble = nengo.Probe(posXY_ensemble)
                self.probesSetPointEnsemble = nengo.Probe(setPointEnsemble, synapse=0.1)
                self.probesErrorX = nengo.Probe(errorX, synapse=0.1)
                self.probesErrorY = nengo.Probe(errorY, synapse=0.1)
                self.probesd_e_x = nengo.Probe(d_e_x, synapse=0.1)
                self.probesd_e_y = nengo.Probe(d_e_y, synapse=0.1)
                self.probesu = nengo.Probe(u, synapse=0.1)
        
        
    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    def get_model(self): 
        return self.model
    def get_probes(self): 
        return [self.probesPosXYEnsemble, self.probesSetPointEnsemble, 
                self.probesErrorX, self.probesErrorY, self.probesd_e_x, self.probesd_e_y, self.probesu]
    

class touchScreenCoordinates: 
    def __init__(self): 
        rospy.init_node('PIDNengo', anonymous=True)
        self.x = 0
        self.y = 0
        self.Ix = np.array([0, 0])
        self.Iy = np.array([0, 0])
        c = Constants()
        self.kPID = c.PIDConstants()
        

        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=10)
        
        self.rate = rospy.Rate(240)
        
    def callback(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        global pop 
        pop = True
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
        #self.tik = time.time()
        return self.handle_output()
    def handle_output(self): 

        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
            #self.Ix, self.Iy = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            # self.Ix[-1] = self.xyFiltered[0]
            # self.Iy[-1] = self.xyFiltered[1]

        #tok = time.time()
        #print(tok-self.tik)
        return [self.Ix[-1], self.Iy[-1]] #[self.x, self.y] #
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        # Ix[-1] = np.sum(Ix[-kernelSize:])/kernelSize
        # Iy[-1] = np.sum(Iy[-kernelSize:])/kernelSize
        # dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        # dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]#Ix, Iy

class runModel: 
    def __init__(self):
        
        global pop
        self.timeSeries = np.array([])
        p = PIDNengo()
        model = p.get_model()
        sim = nengo.Simulator(model, dt=0.005, optimize=True)
        
        
        probes = p.get_probes()
        self.start_time = rospy.Time.now()
        try:
            #if pop:
            for i in range(5000): 
                self.current_time = (rospy.Time.now() - self.start_time).to_sec() #(time.time() - self.start_time) #(rospy.Time.now() - self.start_time).to_sec()
                self.timeSeries = np.append(self.timeSeries, self.current_time)
                sim.step()
                pop = False
                # tik = time.time()
                # tok = time.time()
                # print(tok - tik)
            dataProbe0 = sim.data[probes[0]]
            dataProbe1 = sim.data[probes[1]]
            dataProbe2 = sim.data[probes[2]]
            dataProbe3 = sim.data[probes[3]]
            dataProbe4 = sim.data[probes[4]]
            dataProbe5 = sim.data[probes[5]]
            dataProbe6 = sim.data[probes[6]]
        except SystemExit:
            pass
        saveArray(dataProbe0[:, 0], dataProbe0[:, 1], self.timeSeries, 'touchScreenReadingNengo')
        saveArray(dataProbe1[:, 0], dataProbe1[:, 1], self.timeSeries, 'setPointEnsembleNengo')
        saveArray1(dataProbe2[:, 0], self.timeSeries, 'errorNengoX')
        saveArray1(dataProbe3[:, 0], self.timeSeries, 'errorNengoY')
        saveArray1(dataProbe4[:, 0], self.timeSeries, 'derivativeXNengo')
        saveArray1(dataProbe5[:, 0], self.timeSeries, 'derivativeYNengo')
        saveArray(dataProbe6[:, 0], dataProbe6[:, 1], self.timeSeries, 'controlNengo')
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