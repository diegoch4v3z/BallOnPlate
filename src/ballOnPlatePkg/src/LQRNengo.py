#! /usr/bin/env python3
# Diego Chavez Arana 
# Omar Garcia 
# Ignacio Rubio Scola 
# New Mexico State University

import nengo, rospy, sys
import matplotlib.pyplot as plt 
from constants import Constants
from std_msgs.msg import Float32MultiArray
from plots import plotTwoAxis, saveArray, saveArray1
import numpy as np
import scipy, datetime, time
import message_filters
#import control as ct


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
class LQRNengo: 
    def __init__(self): 
        # Import PID Nengo Constants 
        c = Constants()
        self.kPID = c.PIDNengoConstants()
        self.i = 0
        self.buildNengoModel(probe = False)
    def buildNengoModel(self, probe = False): 
        
        # Constants 
        dt = 1e-3
        gravity = 9.81
        t_synapse = 0.2
        r = 0.1
        radC=0.05

        Ap=np.array([[0,1],[0,0]])
        Bp=np.array([[0],[-5/7*9.81]])
        Cpy=np.array([[1,0]])


        Qc=np.diag([10,0.01])
        Rc=np.eye(1)/10
        #Lc,Sc,Ec=ct.lqr(Ap,Bp,Qc,Rc)
        #Kx=-Lc

        # Kx=np.array([[1.143,1.2]])

        Qo=np.eye(2)*100
        Ro=np.eye(1)/10
        #Lo,So,Eo=ct.lqr(Ap.T,Cpy.T,Qo,Ro)
        
        #Lx=-Lo.T

        # Ac=Ap+Bp@Kx+Lx@Cpy
        # Bc=-Lx
        # Cc=Kx
        # Ac=np.array([[ -4.47224776,   1.        ],
        #             [-11.80923651,  -2.29240951]])

        # Bc=np.array([[ 4.47224776],
        #             [10.        ]])

        # Cc=np.array([[0.25819889, 0.32715324]])


        # Ac=np.array([[ -4.47224776,   1.],[-19.90959646,  -8.3017615 ]])
        # Bc=np.array([[ 4.47224776],[10.]])
        # Cc=np.array([[1.41421356, 1.18475699]])

        Ac=np.array([[-2.70639157,  1.        ], [-4.97151417, -6.02924874]])
        Bc=np.array([[2.70639157],[3.16227766]])
        Cc=np.array([[0.25819889, 0.86044325]])


        # Network model

        self.model = nengo.Network(label='PID', seed=1)
            
        NType= nengo.LIF()
        dt = 0.005
        delayX = DelayX(1, timesteps=int(0.05 / 0.005))
        delayY = DelayY(1, timesteps=int(0.05 / 0.005))
        with self.model:
            # Nodes
            self.PIDNengoNode = touchScreenCoordinates()
            posXY_node = nengo.Node(self.PIDNengoNode, size_out=2, size_in=2)
            
                # Ensembles to represent matrices Ac and Bc
            fcx = nengo.Ensemble(n_neurons=1000, dimensions=2, radius = radC) # Estados PLOT
            fcy = nengo.Ensemble(n_neurons=1000, dimensions=2, radius = radC) # Estados PLOT

                #funcion de dinamica del controlador
            def fc_fun(x):
                # x= xc1,xc2
                return ((Ac@x.reshape(2,1))*t_synapse+x.reshape(2,1)).flatten()
            
                #funcion de entrada del controlador
            def gc_fun(t,x):
                yp=x[0].reshape(1,1)
                return ((Bc@yp)*t_synapse).flatten()

            def satfun(x,a):
                return np.clip(x,-a,a)
            #funcion de salida del controlador
            def control(t,x):
            # x= xc1,xc2
                return satfun((Cc@x.reshape(2,1)).flatten(),0.08)
            # #funcion de salida del controlador
            # def control(t,x):
            #     # x= xc1,xc2
            #     return (Cc@x.reshape(2,1)).flatten()
        
            # Referencias de posicion
        
            def reffunx(t):
                if t<0:
                    return 0
                else:
                    return r*np.sin(0.25*(t))
            def reffuny(t):
                if t<0:
                    return 0
                else:
                    return r*np.cos(0.25*(t))

            gcx=nengo.Node(output=gc_fun,size_in=1, size_out=2) #Nodo de entrada del controlador
            nengo.Connection(gcx,fcx,synapse = t_synapse)  #Entrada del controlador
            nengo.Connection(fcx,fcx,synapse = t_synapse,function=fc_fun) #Dinamica del controlador

            ycx=nengo.Node(output=control,size_in=2, size_out=1) #Nodo de salida del controlador
            nengo.Connection(fcx,ycx,synapse = 0) #Salida del controlador
    
            gcy=nengo.Node(output=gc_fun,size_in=1, size_out=2) #Nodo de entrada del controlador
            nengo.Connection(gcy,fcy,synapse = t_synapse) #Entrada del controlador
            nengo.Connection(fcy,fcy,synapse = t_synapse,function=fc_fun)#Dinamica del controlador
    
            ycy=nengo.Node(output=control,size_in=2, size_out=1)#Nodo de salida del controlador
            nengo.Connection(fcy,ycy,synapse = 0)#Salida del controlador

            RefX = nengo.Node(lambda t,x: reffunx(t),size_in=1, size_out=1)# Nodo de referencia para los plots
            RefY = nengo.Node(lambda t,x: reffuny(t),size_in=1, size_out=1)# Nodo de referencia para los plots
        
            # Errores de posicion al controlador     
            ErrRefX = nengo.Node(lambda t,x: x-0,size_in=1, size_out=1) # Error de posicion
            ErrRefY = nengo.Node(lambda t,x: x-0,size_in=1, size_out=1) # Error de posicion

            nengo.Connection(posXY_node[0],ErrRefX, synapse=0)
            nengo.Connection(posXY_node[1],ErrRefY, synapse=0)
            nengo.Connection(ErrRefX,gcx,synapse = 0) # Conexión de la segunda entrada del controlador
            nengo.Connection(ErrRefY,gcy,synapse = 0)

            ConvertX = nengo.Node(lambda t,x: np.degrees(np.arcsin(x)), size_in=1, size_out=1)
            ConvertY = nengo.Node(lambda t,x: np.degrees(np.arcsin(x)), size_in=1, size_out=1)

            nengo.Connection(ycx, ConvertX, synapse=0)
            nengo.Connection(ycy, ConvertY, synapse=0)
            

            nengo.Connection(ConvertX, posXY_node[0], synapse=0, transform=-1)
            nengo.Connection(ConvertY, posXY_node[1], synapse=0, transform=-1)
            if probe: 
                print('Probing')
                
        
        
    def runNode(self): 
        try: 
            while not rospy.is_shutdown(): 
                self.rate.sleep()
        except rospy.ROSInterruptException: 
            pass 
    def get_model(self): 
        return self.model
    def get_probes(self): 
        return [False]
    

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
        return self.handle_output()
    def handle_output(self): 

        self.Ix = np.append(self.Ix, self.x)
        self.Iy = np.append(self.Iy, self.y)
        if len(self.Ix) > self.kPID[7] and len(self.Iy) > self.kPID[7]: 
            self.xyFiltered = self.movingAverage(self.Ix, self.Iy, self.kPID[7],self.kPID[8])
            self.Ix[-1] = self.xyFiltered[0]
            self.Iy[-1] = self.xyFiltered[1]
        return [self.Ix[-1], self.Iy[-1]] #[self.x, self.y] #
    def movingAverage(self, Ix, Iy, kernelSize, kernelDelay): 
        kernel = np.ones(kernelSize)/kernelSize
        dataConvolvedX = np.convolve(Ix[-kernelSize:], kernel, mode = 'same')
        dataConvolvedY = np.convolve(Iy[-kernelSize:], kernel, mode = 'same')
        x = dataConvolvedX[kernelDelay]
        y = dataConvolvedY[kernelDelay]
        return [x, y]#Ix, Iy
    
    def returnXYValues(self):
        [self.Ix, self.Iy]
    

class runModel: 
    def __init__(self):
        
        global pop
        self.timeSeries = np.array([])
        p = LQRNengo()
        model = p.get_model()
        sim = nengo.Simulator(model, dt=0.001, optimize=True)
        
        
        #probes = p.get_probes()
        self.start_time = rospy.Time.now()
        try:
            #if pop:
            for i in range(5000): 
                self.current_time = (rospy.Time.now() - self.start_time).to_sec() #(time.time() - self.start_time) #(rospy.Time.now() - self.start_time).to_sec()
                self.timeSeries = np.append(self.timeSeries, self.current_time)
                sim.step()
                pop = False
            
            # dataProbe0 = sim.data[probes[0]]
            # dataProbe1 = sim.data[probes[1]]
            # dataProbe2 = sim.data[probes[2]]
            # dataProbe3 = sim.data[probes[3]]
            # dataProbe4 = sim.data[probes[4]]
            # dataProbe5 = sim.data[probes[5]]
            # dataProbe6 = sim.data[probes[6]]
        except SystemExit:
            pass
        # saveArray(dataProbe0[:, 0], dataProbe0[:, 1], self.timeSeries, 'touchScreenReadingNengo')
        # saveArray(dataProbe1[:, 0], dataProbe1[:, 1], self.timeSeries, 'setPointEnsembleNengo')
        # saveArray1(dataProbe2[:, 0], self.timeSeries, 'errorNengoX')
        # saveArray1(dataProbe3[:, 0], self.timeSeries, 'errorNengoY')
        # saveArray1(dataProbe4[:, 0], self.timeSeries, 'derivativeXNengo')
        # saveArray1(dataProbe5[:, 0], self.timeSeries, 'derivativeYNengo')
        # saveArray(dataProbe6[:, 0], dataProbe6[:, 1], self.timeSeries, 'controlNengo')
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