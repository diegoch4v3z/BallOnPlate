#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# New Mexico State University

import nengo, rospy, sys
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis, saveArray, saveArray1, saveTimeArray, saveArrayACC
import numpy as np
import scipy, datetime, time, os
import message_filters, nengo_loihi
import nengo_loihi.hardware as hardware


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

# the object PIDNengo when initialized only calls to the touchScreenCoordinates 
# class which contains the callback function that takes the information from the touchscreen
# and sends it to the handleouput that sends info to nengo. 
# nengo calls the sim.step to handle that output and send back info to the 
# servo through the __call__ function. 
# The sim.step should be in the callback function of the 

class PIDNengo: 
    def __init__(self): 
        # import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        self.kPIDn = c.PIDConstants()
        # build Nengo model
        self.buildNengoModel(probe = True)
        #self.sim = nengo_loihi.Simulator(self.model, dt=0.008, progress_bar=False, target='loihi', hardware_options={"snip_max_spikes_per_step": 300, "n_chips": 2})
        #self.sim = nengo_loihi.Simulator(self.model, dt=0.008, progress_bar=False, target='sim')
        self.sim = nengo.Simulator(self.model, dt=0.008, progress_bar=False, optimize=True)
        self.probes = self.get_probes()
        self.queueSize = 10
        # initialize rospy 
        
        rospy.init_node('PIDNengo', anonymous=True)
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=1)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=self.queueSize)
        self.rate = rospy.Rate(240)

        # data storage arrays
        self.x = 0              # information from the touchscreen will be stored on x and y 
        self.y = 0
        self.Ix = np.array([0, 0])
        self.Iy = np.array([0, 0])
        self.timeSeries = np.array([]) 
        self.frequency = np.array([])
        self.globalTimeSeries = np.array([])
        self.globalTime = rospy.Time.now() 
        self.start_time = rospy.Time.now()
        self.tn = np.array([])
        self.t = rospy.Time.now()

        # nengo Loihi 
        os.environ.update({'KAPOHOBAY': '1'})
        nengo_loihi.set_defaults()
        
    def __call__(self, t, values): 
        servoData = Float32MultiArray()
        servoData.data = [values[0], values[1]]
        self.pubServo.publish(servoData)
        #rospy.loginfo("Published servo control: %s", values)
        self.rate.sleep()
        return self.handle_output()
    def handle_output(self): 
        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPIDn[7],self.kPIDn[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
        return [self.Ix[-1], self.Iy[-1]]

    def callback(self, msg):
        #self.globalTime = (rospy.Time.now() - self.start_time) - self.globalTime
        #print(self.globalTime)
        #rospy.loginfo("Received Touchscreen Coordinates: %s", str(msg.data))
        self.tn = np.append(self.tn, (rospy.Time.now() - self.t).to_sec())
   
        self.currentTime = rospy.Time.now()
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.current_time = (rospy.Time.now() - self.start_time).to_sec()
        self.timeSeries = np.append(self.timeSeries, self.current_time)
        self.sim.step()
        self.deltaTime = (rospy.Time.now() - self.currentTime).to_sec()
        self.frequency = np.append(self.frequency, 1/self.deltaTime)
    

    # def __call__(self, t, values): 
    #     servoData = Float32MultiArray()
    #     servoData.data = [values[0], values[1]]
    #     #self.pubServo.publish(servoData)
    #     self.rate.sleep()
    #     return self.handle_output()
    
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]
     
    def buildNengoModel(self, probe = False): 
        self.model = nengo.Network(label='PID', seed=1)
        NType= nengo_loihi.LoihiLIF()
        dt = 0.005
        delayX = DelayX(1, timesteps=int(0.05 / 0.005))
        delayY = DelayY(1, timesteps=int(0.05 / 0.005))
        with self.model:
            # Nodes
            posXY_node = nengo.Node(self.__call__, size_out=2, size_in=2)
            setpoint = nengo.Node([0, 0])
            delayNodeX = nengo.Node(delayX.step, size_in=1, size_out=1)
            delayNodeY = nengo.Node(delayY.step, size_in=1, size_out=1)
            bias = nengo.Node([0, 0]) 

            #Ensembles
            posXY_ensemble = nengo.Ensemble(n_neurons=100, dimensions=2, neuron_type=nengo.Direct(), radius=1.5, label='posXY_ensemble') 
            setPointEnsemble = nengo.Ensemble(n_neurons=400, dimensions=2, radius = 0.1, neuron_type=NType,  label='setPointEnsemble')
            errorX = nengo.Ensemble(n_neurons=400, dimensions=1, radius=2, neuron_type=NType, label='errorX')
            errorY = nengo.Ensemble(n_neurons=400, dimensions=1, radius=2, neuron_type=NType, label='errorY')
            d_e_x = nengo.Ensemble(n_neurons=800, dimensions = 1, radius=1, neuron_type=NType, label='d_e_x')#radius=1)
            d_e_y = nengo.Ensemble(n_neurons=800, dimensions = 1, radius=1, neuron_type=NType, label='d_e_y')#radius=1
            u = nengo.Ensemble(n_neurons=1, dimensions=2, neuron_type=nengo.Direct(), radius=1, label='u') #bias=[0.5], gain=[1.0])
            

            nengo.Connection(bias[0], u[0], synapse=None)

            biasuProbe = nengo.Node([0, 0]) #[0.09, 0]
            uProbe = nengo.Ensemble(n_neurons=1, dimensions=2, neuron_type=nengo.Direct(), radius=1) 
            nengo.Connection(u[0], uProbe[0], synapse=None)
            nengo.Connection(biasuProbe[0], uProbe[0], synapse=None)
            nengo.Connection(u[1], uProbe[1], synapse=None)
            nengo.Connection(biasuProbe[1], uProbe[1], synapse=None)
            
            

            # Conections 
            nengo.Connection(setpoint, setPointEnsemble, synapse=None)
            nengo.Connection(posXY_node, posXY_ensemble, synapse=None)
            nengo.Connection(setPointEnsemble[0], errorX, synapse=None, transform=1.0)#synapse=nengo.Alpha(tau=0.01), transform=3)
            nengo.Connection(setPointEnsemble[1], errorY, synapse=None, transform=1.0)
            nengo.Connection(posXY_ensemble[0], errorX, synapse=None, transform=-1.0) #synapse=nengo.Alpha(tau=0.01), transform=-3)
            nengo.Connection(posXY_ensemble[1], errorY, synapse=None, transform=-1.0)

            # Derivative in X
            nengo.Connection(errorX, delayNodeX)
            nengo.Connection(delayNodeX, d_e_x, transform=20) # 20
            nengo.Connection(errorX, d_e_x, transform=-20)
            # Derivative in Y 
            nengo.Connection(errorY, delayNodeY)
            nengo.Connection(delayNodeY, d_e_y, transform=20)
            nengo.Connection(errorY, d_e_y, transform=-20)
            nengo.Connection(errorX, u[0], transform=self.kPID[0]) 
            nengo.Connection(errorY, u[1], transform=self.kPID[3]) 
            nengo.Connection(d_e_x, u[0], transform=self.kPID[2])
            nengo.Connection(d_e_y, u[1], transform=self.kPID[5])
            nengo.Connection(u, posXY_node, synapse=0.03)#0.05)#synapse=0.005)
            if probe: 
                self.probesPosXYEnsemble = nengo.Probe(posXY_ensemble)
                self.probesSetPointEnsemble = nengo.Probe(setPointEnsemble, synapse=0.1)
                self.probesErrorX = nengo.Probe(errorX, synapse=0.1)
                self.probesErrorY = nengo.Probe(errorY, synapse=0.1)
                self.probesd_e_x = nengo.Probe(d_e_x, synapse=0.1)
                self.probesd_e_y = nengo.Probe(d_e_y, synapse=0.1)
                self.probesu = nengo.Probe(u, synapse=0.1)
                self.probesuNot = nengo.Probe(uProbe, synapse=0.1)
                #self.PWeights = nengo.Probe(conn1, "weights", sample_every=0.01)
    def run(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass     
        
    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    def get_model(self): 
        return self.model
    def get_probes(self): 
        return [self.probesPosXYEnsemble, self.probesSetPointEnsemble, self.probesErrorX, self.probesErrorY, self.probesd_e_x, self.probesd_e_y, self.probesuNot] #self.PWeights]
    
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
    def __call__(self, t, values): 
        servoData = Float32MultiArray()
        servoData.data = [values[0], values[1]]
        #self.pubServo.publish(servoData)
        self.rate.sleep()
        return self.handle_output()
    def handle_output(self): 
        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]

        return [self.Ix[-1], self.Iy[-1]]
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]
    
    def returnXYValues(self):
        [self.Ix, self.Iy]
    


def send_interrupt_to_other_code():
    # Assuming the other code's process name is "other_code.py"
    import os
    process_name = "PID.py"
    process_name1 = "touchscreen.py"
    os.system("pkill -SIGINT -f {}".format(process_name))
    os.system("pkill -SIGINT -f {}".format(process_name1))

if __name__ == '__main__':
    p = PIDNengo()
    try: 
        while not rospy.is_shutdown(): 
            p.run()
        probes = p.get_probes()
        sim = p.sim
        timeseries = p.timeSeries
        frequency = p.frequency
        dataProbe0 = sim.data[probes[0]]
        dataProbe1 = sim.data[probes[1]]
        dataProbe2 = sim.data[probes[2]]
        dataProbe3 = sim.data[probes[3]]
        dataProbe4 = sim.data[probes[4]]
        dataProbe5 = sim.data[probes[5]]
        dataProbe6 = sim.data[probes[6]]
        #weigths = sim.data[probes[7]]
        saveArray(dataProbe0[:, 0], dataProbe0[:, 1], timeseries, 'touchScreenReadingNengo')
        saveArray(dataProbe1[:, 0], dataProbe1[:, 1], timeseries, 'setPointEnsembleNengo')
        saveArray(dataProbe2[: ,0], dataProbe3[:, 0], timeseries, 'errorNengo')
        saveArray(dataProbe4[:, 0], dataProbe5[:, 0], timeseries, 'derivativeNengo')
        saveArray(dataProbe6[:, 0], dataProbe6[:, 1], timeseries, 'controlNengo')
        saveArray(frequency, frequency, timeseries, 'frequencyNengo')
        saveArray(p.tn, p.tn, timeseries, 'measurementTimeNengo')
        #saveArray(weigths, weigths, weigths, 'weightsNengo')
        send_interrupt_to_other_code()
    except rospy.ROSInterruptException: 
        pass