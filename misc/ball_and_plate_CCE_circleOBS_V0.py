
#This code simulates the dynamics of the ball and plate with a feedback state
#control using spiking neural networks

import matplotlib.pyplot as plt
#%matplotlib inline

plt.close("all")

import nengo
import numpy as np
import scipy as sp
import nengo_loihi
import control as ct

dt=1e-3

gravity = 9.81

t_synapse = 0.2

r = 0.1;

radC=0.3

# b=np.array([0.0871,7.97])


Ap=np.array([[0,1],[0,0]])
Bp=np.array([[0],[-5/7*9.81]])
Cpy=np.array([[1,0]])


Qc=np.diag([10,0.01])
Rc=np.eye(1)/10
Lc,Sc,Ec=ct.lqr(Ap,Bp,Qc,Rc)
Kx=-Lc

# Kx=np.array([[1.143,1.2]])

Qo=np.eye(2)*100
Ro=np.eye(1)/10
Lo,So,Eo=ct.lqr(Ap.T,Cpy.T,Qo,Ro)
Lx=-Lo.T

Ac=Ap+Bp@Kx+Lx@Cpy
Bc=-Lx
Cc=Kx


def FC():
    with nengo.Network(label="FullController") as Fullcontrol:
    
        #Controllers
        #Two ensembles are created to represent the matrices Ac and Bc
        
        Fullcontrol.fcx = nengo.Ensemble(n_neurons=1000, dimensions=2, radius = radC)
        Fullcontrol.fcy = nengo.Ensemble(n_neurons=1000, dimensions=2, radius = radC)
        
        #The functions Ac_fun and Bc_fun model the terms fc'=TAcx and gc'=TBcu. 
    
        #funcion de dinamica del controlador
        def fc_fun(x):
            # x= xc1,xc2
            return ((Ac@x.reshape(2,1))*t_synapse+x.reshape(2,1)).flatten()
            
        #funcion de entrada del controlador
        def gc_fun(t,x):
            yp=x[0].reshape(1,1)
            return ((Bc@yp)*t_synapse).flatten()
        
        #funcion de salida del controlador
        def control(t,x):
            # x= xc1,xc2
            return (Cc@x.reshape(2,1)).flatten()
        
        Fullcontrol.gcx=nengo.Node(output=gc_fun,size_in=1, size_out=2) #Nodo de entrada del controlador
        nengo.Connection(Fullcontrol.gcx,Fullcontrol.fcx,synapse = t_synapse)  #Entrada del controlador
        nengo.Connection(Fullcontrol.fcx,Fullcontrol.fcx,synapse = t_synapse,function=fc_fun) #Dinamica del controlador
    
        Fullcontrol.ycx=nengo.Node(output=control,size_in=2, size_out=1) #Nodo de salida del controlador
        nengo.Connection(Fullcontrol.fcx,Fullcontrol.ycx,synapse = 0) #Salida del controlador
    
        Fullcontrol.gcy=nengo.Node(output=gc_fun,size_in=1, size_out=2) #Nodo de entrada del controlador
        nengo.Connection(Fullcontrol.gcy,Fullcontrol.fcy,synapse = t_synapse) #Entrada del controlador
        nengo.Connection(Fullcontrol.fcy,Fullcontrol.fcy,synapse = t_synapse,function=fc_fun)#Dinamica del controlador
    
        Fullcontrol.ycy=nengo.Node(output=control,size_in=2, size_out=1)#Nodo de salida del controlador
        nengo.Connection(Fullcontrol.fcy,Fullcontrol.ycy,synapse = 0)#Salida del controlador
        
    
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
        
        Fullcontrol.RefX = nengo.Node(lambda t,x: reffunx(t),size_in=1, size_out=1)# Nodo de referencia para los plots
        Fullcontrol.RefY = nengo.Node(lambda t,x: reffuny(t),size_in=1, size_out=1)# Nodo de referencia para los plots
        
        # Errores de posicion al controlador     
        Fullcontrol.ErrRefX = nengo.Node(lambda t,x: x-reffunx(t),size_in=1, size_out=1) # Error de posicion
        Fullcontrol.ErrRefY = nengo.Node(lambda t,x: x-reffuny(t),size_in=1, size_out=1) # Error de posicion
    
        nengo.Connection(Fullcontrol.ErrRefX,Fullcontrol.gcx,synapse = 0) # Conexión de la segunda entrada del controlador
        nengo.Connection(Fullcontrol.ErrRefY,Fullcontrol.gcy,synapse = 0) # Conexión de la segunda entrada del controlador
        

        return Fullcontrol
    

def BallPlate_toInt(t,x,u):
    return np.array([x[1],-u[0]*5/7*gravity,x[3],-u[1]*5/7*gravity]).flatten()

def BallandPlate(t,X):
    x=X[:4]
    u=X[4:]
      
    if t<=dt:
        X=np.zeros((4))
        X[0]=0
        return (X).flatten()
    else:   
        sol=sp.integrate.solve_ivp(lambda t,x:BallPlate_toInt(t,x,u), (t,t+dt), x)
        return ((sol.y)[:,-1]).flatten()



def BP():
    with nengo.Network(label="BallandPlate") as BallPlate:
            
        BallPlate.Model = nengo.Node(output=BallandPlate, size_in=6, size_out=4)

        nengo.Connection(BallPlate.Model, BallPlate.Model[:4], transform=1, synapse=dt*0)
        BallPlate.Model_Probes = nengo.Probe(BallPlate.Model, synapse=dt*0)  
    
        return BallPlate
 
    
net = nengo.Network(label="Closed Loop")
with net:
    net.BallPlate=BP()
    net.Fullcontrol=FC()
    
    #Entradas del sistema
    ux=net.BallPlate.Model[4]
    uy=net.BallPlate.Model[5]
    #Salidas del sistema
    ypx=net.BallPlate.Model[0]
    ypy=net.BallPlate.Model[2]
    
    
    #Entradas del Controlador
    ErX=net.Fullcontrol.ErrRefX
    ErY=net.Fullcontrol.ErrRefY
    
    #Salidas del controlador
    ucx=net.Fullcontrol.ycx
    ucy=net.Fullcontrol.ycy
    
    nengo.Connection(ucx,ux,synapse = 0)
    nengo.Connection(ucy,uy,synapse = 0)
    
    nengo.Connection(ypx,ErX,synapse = 0)
    nengo.Connection(ypy,ErY,synapse = 0)
    
    ref_probeX = nengo.Probe(net.Fullcontrol.RefX,synapse=0.1) #Probes
    ref_probeY = nengo.Probe(net.Fullcontrol.RefY,synapse=0.1)
        
    # error_probe = nengo.Probe(Error,synapse=0.1)
    # dex_probe = nengo.Probe(d_e_x,synapse=0.1)
    # dey_probe = nengo.Probe(d_e_y,synapse=0.1)
    # ref_Ens_probe = nengo.Probe(Encoded_Reference,synapse=0.1)
    #Pos_x_and_y_probe = nengo.Probe(Pos_x_and_y,synapse=0.1)
    ens_probe = nengo.Probe(net.BallPlate.Model,synapse=0.1)
    # g_probe = nengo.Probe(g, synapse = 0.1)


with nengo.Simulator(net,dt=dt, progress_bar=False) as sim:
    sim.run(120)
t = sim.trange()
data = sim.data
print(t)


#X axis
plt.figure(0)
plt.plot(t,data[ens_probe][:,0])
plt.plot(t,data[ref_probeX])
plt.xlabel("$t$", fontsize="x-large")
plt.ylabel("$x$", fontsize="x-large")
plt.grid(True)

#Y axis
plt.figure(1)
plt.plot(t,data[ens_probe][:,2])
plt.plot(t,data[ref_probeY])
plt.xlabel("$t$", fontsize="x-large")
plt.ylabel("$y$", fontsize="x-large")
plt.grid(True)

# #     #States x and y
plt.figure(2)
plt.plot(data[ens_probe][:,0], data[ens_probe][:,2])
plt.plot(data[ref_probeX],data[ref_probeY])
plt.xlabel("$x$", fontsize="x-large")
plt.ylabel("$y$", fontsize="x-large")
plt.grid(True)
