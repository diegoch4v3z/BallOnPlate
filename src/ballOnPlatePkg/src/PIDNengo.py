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


class PIDNengo: 
    def __init__(self): 
        # import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        # ROS
        rospy.init_node('PIDNengo', anonymous=True)
        self.pubServo = rospy.Publisher('servoData', Float32MultiArray, queue_size=1)
        self.rate = rospy.Rate(240)
        # Nengo model
        self.buildNengoModel()
        self.sim = nengo.Simulator(self.model, dt=0.008, progress_bar=False, optimize=True)
        
        self.sub = rospy.Subscriber('touchscreenData', Float32MultiArray, callback=self.callback, queue_size=1)

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
        return self.handle_output()
    
    def handle_output(self): 
        return [self.x, self.y]

    def callback(self, msg):
        #rospy.loginfo("Received Touchscreen Coordinates: %s", str(msg.data))
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.sim.step()
    
    def outputControl(self, t, x):
    
        servoData = Float32MultiArray()
        servoData.data = [x[0], x[1]]
        self.pubServo.publish(servoData)
        #rospy.loginfo("Published servo control: %s", x)
    
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]
    
    def step(self, t, values): 
        if t < 5.0: 
            return [0, 0]
        elif t>5.0 and t < 5.10: 
            return [0.0, 0.9]
        elif t>10.0 and t < 10.10:
            return [0.0, 0.9]
        elif t>15.0 and t < 15.10:
            return [0.0, 0.9]
        else:
            return [0, 0]
     
    def buildNengoModel(self, probe = True): 
        self.model = nengo.Network(label='PID', seed=1)
        NType= nengo_loihi.LoihiLIF()
        
        
        delayX = DelayX(1, timesteps=int(0.05 / 0.005))
        delayY = DelayY(1, timesteps=int(0.05 / 0.005))
        with self.model:
            # Nodes
            posXY_node = nengo.Node(self.__call__, size_out=2, size_in=2)
            U_node = nengo.Node(self.outputControl, size_in=2)
            setpoint = nengo.Node([0, 0])
            delayNodeX = nengo.Node(delayX.step, size_in=1, size_out=1)
            delayNodeY = nengo.Node(delayY.step, size_in=1, size_out=1)
            disturbance = nengo.Node(self.step, size_in=2, size_out=2)
            

            #Ensembles 
            errorX = nengo.Ensemble(n_neurons = 400, dimensions = 1, radius=2, neuron_type=NType, label='errorX', normalize_encoders=True)
            errorY = nengo.Ensemble(n_neurons = 400, dimensions = 1, radius=2, neuron_type=NType, label='errorY', normalize_encoders=True)
            d_e_x = nengo.Ensemble(n_neurons = 400, dimensions = 1, radius=2, neuron_type=NType, label='d_e_x')
            d_e_y = nengo.Ensemble(n_neurons = 400, dimensions = 1, radius=2, neuron_type=NType, label='d_e_y')
            u = nengo.Ensemble(n_neurons=1, dimensions=2, neuron_type=nengo.Direct(), radius=1, label='u')

            nengo.Connection(setpoint[0], errorX, synapse=None, transform=-1.0)
            nengo.Connection(setpoint[1], errorY, synapse=None, transform=-1.0)
            nengo.Connection(posXY_node[0], errorX, synapse=None, transform=1.0)
            nengo.Connection(posXY_node[1], errorY, synapse=None, transform=1.0)
            nengo.Connection(disturbance, u)

            # Derivative in X
            nengo.Connection(errorX, delayNodeX, synapse=None)
            nengo.Connection(delayNodeX, d_e_x, transform=-10, synapse=0.01)
            nengo.Connection(errorX, d_e_x, transform=10, synapse=0.01)
            # Derivative in Y 
            nengo.Connection(errorY, delayNodeY, synapse=None)
            nengo.Connection(delayNodeY, d_e_y, transform=-10, synapse=0.01)
            nengo.Connection(errorY, d_e_y, transform=10, synapse=0.01)
            nengo.Connection(errorX, u[0], transform=self.kPID[0]) 
            nengo.Connection(errorY, u[1], transform=self.kPID[3]) 
            nengo.Connection(d_e_x, u[0], transform=self.kPID[2])
            nengo.Connection(d_e_y, u[1], transform=self.kPID[5])
            nengo.Connection(u, U_node, synapse=0.01)

        
            if probe: 
                ensembles = [posXY_node, errorX, errorY, d_e_x, d_e_y, u, setpoint, disturbance]
                self.probesList = []
                for i in range(len(ensembles)): 
                    self.probesList.append(nengo.Probe(ensembles[i], synapse=0.05))


    def run(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 

    def get_model(self): 
        return self.model
    def get_probes(self): 
        return self.probesList
    def plot(self): 
        namesList = ['Position X/Y','Error X', 'Error Y', 'Derivative X', 'Derivative Y', 'Control U', 'setpoint', 'disturbance']
        namesSubList = {'Position X/Y': ['Position X', 'Position Y'], 'Control U': ['Control UX', 'Control UY'], 'setpoint': ['Setpoint X', 'Setpoint Y'], 'disturbance': ['Disturbance X', 'Disturbance Y']}
        fig, axs = plt.subplots(len(self.probesList), 1, figsize=(15, 10))
        for i in range(len(self.probesList)):
            print(self.probesList[i])
            if np.shape(self.sim.data[self.probesList[i]])[1] == 1:
                axs[i].plot(self.sim.trange(), self.sim.data[self.probesList[i]], label='Ensemble: ' + namesList[i])
            else:
                for j in range(2):
                    axs[i].plot(self.sim.trange(), self.sim.data[self.probesList[i]][:, j], label='Ensemble: ' + namesSubList[namesList[i]][j])
            axs[i].set_xlabel('Time [s]')
            axs[i].set_ylabel('Value')
            axs[i].legend()
            axs[i].grid()
        plt.tight_layout()
        plt.show()
        plt.savefig('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/PIDNengo_Probed_Ensemble.png', dpi=300)
        plt.close()
        np.savetxt('/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src/PIDNengo_Probed_Timestamp.txt', self.sim.trange())
    def plotACC_X(self): 
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = 'Times New Roman'
        plt.rcParams['pdf.fonttype'] = 42

        fig, (ax1, ax2) = plt.subplots(2) 
        ax1.plot(self.sim.trange(), self.sim.data[self.probesList[5]][:, 0], label='ux', color = 'red')
        ax1.set_ylabel('ux (rad)', color = 'black', fontsize=14)
        ax1.set_yticks([-0.8, -0.4, 0, 0.4, 0.8])

        ax3 = ax1.twinx()
        ax3.plot(self.sim.trange(), self.sim.data[self.probesList[0]][:, 0]+0.05, label='x', color = 'blue')
        ax3.plot(self.sim.trange(), self.sim.data[self.probesList[6]][:, 0], label='xdes', color = 'black')
        ax3.set_ylabel('x, xd (mm)', fontsize=14)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines3, labels3 = ax3.get_legend_handles_labels()
        ax1.legend(lines1 + lines3, labels1 + labels3, loc='upper right')

        ax3.set_ylim(-1.0, 1.0) 
        ax1.set_ylim(-0.8, 0.8) 
        ax1.set_xlim(0, 20)
        ax2.set_xlim(0, 20)
        
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[1]] + 0.03, label='error', color = 'blue')
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[6]][:, 0], label='xdes', color = 'black')
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[7]][:, 0], label='impulse', color = 'red')

        ax2.set_ylim(-0.8, 0.8) 
        ax2.set_yticks([-0.8, -0.4, 0, 0.4, 0.8])
        ax3.set_xticks([0, 5, 10, 15, 20])
        ax2.set_xticks([0, 5, 10, 15, 20])
        ax2.set_ylabel('error (mm)', fontsize=14)
        ax2.set_xlabel('Time (s)', fontsize=14)

        ax2.legend()
        ax1.grid()
        ax2.grid()

        plt.subplots_adjust(wspace=0, hspace=0)
        plt.tight_layout()

        save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src' 
        file_name = 'PID_Nengo_XaxisPlot_ACC.pdf' 
        dpi_value = 300  
        save_path = f"{save_folder}/{file_name}"
        plt.savefig(save_path, dpi=dpi_value)

    def plotACC_Y(self): 
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = 'Times New Roman'
        plt.rcParams['pdf.fonttype'] = 42

        fig, (ax1, ax2) = plt.subplots(2) 
        ax1.plot(self.sim.trange(), self.sim.data[self.probesList[5]][:, 1]-0.05, label='uy', color = 'red')
        ax1.set_ylabel('uy (rad)', color = 'black', fontsize=14)
        ax1.set_yticks([-0.8, -0.4, 0, 0.4, 0.8])

        ax3 = ax1.twinx()
        ax3.plot(self.sim.trange(), self.sim.data[self.probesList[0]][:, 1], label='y', color = 'blue')
        ax3.plot(self.sim.trange(), self.sim.data[self.probesList[6]][:, 1], label='ydes', color = 'black')
        ax3.set_ylabel('y, yd (mm)', fontsize=14)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines3, labels3 = ax3.get_legend_handles_labels()
        ax1.legend(lines1 + lines3, labels1 + labels3, loc='upper right')

        ax3.set_ylim(-1.0, 1.0) 
        ax1.set_ylim(-0.8, 0.8) 
        ax1.set_xlim(0, 20)
        ax2.set_xlim(0, 20)
        
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[2]] - 0.02, label='error', color = 'blue')
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[6]][:, 1], label='ydes', color = 'black')
        ax2.plot(self.sim.trange(), self.sim.data[self.probesList[7]][:, 1], label='impulse', color = 'red')

        ax2.set_ylim(-0.8, 0.8) 
        ax2.set_yticks([-0.8, -0.4, 0, 0.4, 0.8])
        ax3.set_xticks([0, 5, 10, 15, 20])
        ax2.set_xticks([0, 5, 10, 15, 20])
        ax2.set_ylabel('error (mm)', fontsize=14)
        ax2.set_xlabel('Time (s)', fontsize=14)

        ax2.legend()
        ax1.grid()
        ax2.grid()

        plt.subplots_adjust(wspace=0, hspace=0)
        plt.tight_layout()

        save_folder = '/home/cortana/Ball_On_Plate_ws/src/ballOnPlatePkg/src' 
        file_name = 'PID_Nengo_YaxisPlot_ACC.pdf' 
        dpi_value = 300  
        save_path = f"{save_folder}/{file_name}"
        plt.savefig(save_path, dpi=dpi_value)



if __name__ == '__main__':
    p = PIDNengo()
    p.run()
    p.plot()
    p.plotACC_Y()