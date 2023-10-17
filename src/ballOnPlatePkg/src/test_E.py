import matplotlib.pyplot as plt
import nengo
import nengo_loihi, os
import numpy as np
import control as ctrl
from datetime import datetime


os.environ.update({'KAPOHOBAY': '1'})


initialtime=datetime.now()
print(initialtime)
      
# plt.close('all')
#Comparing SNN vs Euler

#
Loihi=0 #0 for INRC, 1 for Loihi Hardware Emulation, 2 for pure Nengo simulation
if Loihi==0 or Loihi==1 : 
    nengo_loihi.set_defaults()
    

#System dimensions
n=1 #states
m=1 #inputs
p=1 #outputs

#System Matrices
#Lineal TIme Invariable (LTI)
A=np.array([[2]])
B=np.array([[1]])
C=np.array([[1]])

# Multiplier to amplify the integrator input and to reduce the integrator offset
InpMul=100
# Extended System Matrices
Ae=np.vstack((np.hstack((A,np.zeros((n,1)))),np.hstack((InpMul*C,[[0]])))) 
Be=np.vstack((B,np.zeros((m,1))))

#Control computation
poles=np.array([-2,-2])
Ke=ctrl.acker(Ae, Be, poles) 

K=Ke[0,:n]
Ki=-Ke[0,n]

print([K,Ki])

#Sampling Time
dt=0.001
#Simulation Time
Tf = 10
#Initial Conditions
IC=np.array([1])
# Integration state Initial Condition
IIC=np.array([1])

# Integrator parameters
if Loihi==0 or Loihi==1: 
    tau = 0.2 
    neuInt=500
else:
    tau=0.2
    neuInt=500  
    
radInt=InpMul*1.25 

# Pertrubation function
def pert(t):
    if t>=5 and t<15:
        return np.array([0])+t*0    
    elif t>=15 and t<30:
        return np.array([-1])+t*0    
    else:
        return np.array([1])+t*0

with nengo.Network(label="Test PI") as model:
    
    #Discrete time system
    def mysyst(t, X):
        x=X[:n]
        u=X[n:]
        if t==dt:
            x1=IC             #System Initial condition
        else:
            x1=x+dt*((A-B@K)@x+B@u+B@pert(t)) 
        return x1
              

    #Node for the system dynamics
    model.Systdim = nengo.Node(output=mysyst, size_in=n+m, size_out=n)      
    
    # Integrator ensemble
    model.Int = nengo.Ensemble(neuInt, dimensions=1, radius=radInt)
    
    # Connect the population to itself  
    nengo.Connection(model.Int, model.Int,  synapse=tau) 

    # Connect the input taking into account the Multiplier
    nengo.Connection(model.Systdim, model.Int, transform=[[tau*InpMul]], synapse=tau)
     
    #Nodes interconection
    #Connecting the controller output with the system 
    nengo.Connection(model.Int, model.Systdim[1], transform=Ki)
    #Instant feedback to recover previous state in memory     
    nengo.Connection(model.Systdim[0], model.Systdim[0], transform=1, synapse=0) #Synapsis 0 
    
    
    #Sensors to collect data
    model.probe_states = nengo.Probe(model.Systdim, synapse=tau)
    model.probe_Int = nengo.Probe(model.Int, synapse=tau)
        

#In Simulator dt=0.001 by default
if Loihi==1:
    with nengo_loihi.Simulator(model, target='sim', dt=dt, progress_bar=False) as sim:
        sim.run(Tf)    
elif Loihi==2:
    with nengo.Simulator(model,dt=dt, progress_bar=False) as sim:
        sim.run(Tf)
else:        
    with nengo_loihi.Simulator(model, target='loihi', hardware_options={"snip_max_spikes_per_step": 300, "n_chips": 2}) as sim:
        sim.run(Tf)
        
# Simulation data    
baseline_t = sim.trange()
baseline_data = sim.data

Time=baseline_t
Input=baseline_data[model.probe_Int]*Ki
States=baseline_data[model.probe_states]
Int=baseline_data[model.probe_Int]


np.savez("Time", Time=Time)
np.savez("Input", Input=Input)
np.savez("States", States=States)
np.savez("Int", Int=Int)
 
# Time=np.load("Time.npz")['Time']
# Input=np.load("Input.npz")['Input']
# States=np.load("States.npz")['States']
# Int=np.load("Int.npz")['Int']

finaltime=datetime.now()
print(finaltime-initialtime)
#%%
# ===================
# Plots of the simulation
# ===================

plt.figure(0)
plt.plot(baseline_t, baseline_data[model.probe_Int]*Ki,label="SNN")
mypert=baseline_t*0
for i in np.arange(len(baseline_t)):
    mypert[i]=pert(i*dt)

plt.plot(baseline_t, mypert,label="Pert")
plt.grid()
plt.title('Model Inputs')
plt.figure(2)
plt.plot(baseline_t, baseline_data[model.probe_states],label="SNN")
plt.grid()
plt.title('Model States')

plt.figure(3)
plt.plot(baseline_t, baseline_data[model.probe_Int],label="SNN")
plt.grid()
plt.title('Integrator state')


# ===================
# Euler Integration
# ===================
x=np.zeros((n,int(Tf/dt)))
xi=np.zeros((1,int(Tf/dt)))
u=np.zeros((1,int(Tf/dt)))

x[:,0]=IC
xi[:,0]=IIC
u[:,0]=Ki*xi[:,0]

tsimu=np.arange(0,int(Tf/dt)-1)

for i in tsimu:
    x[:,i+1] =x[:,i]+dt*((A-B@K)@x[:,i]+B@u[:,i]+B@pert(i*dt))
    xi[:,i+1]=xi[:,i]+dt*InpMul*x[:,i]
    u[:,i+1] =Ki*xi[:,i+1]

tplot=np.arange(0,int(Tf/dt))    

# ===================
# Plots for Euler simulation
# ===================

plt.figure(0)    
plt.plot(tplot*dt,u.T,'-',label="euler") 
plt.legend() 

plt.savefig('fig1.png')

plt.figure(2)
plt.plot(tplot*dt,x.T,'-',label="euler")
plt.legend()
plt.savefig('fig2.png')

 
plt.figure(3)
plt.plot(tplot*dt,xi.T,'-',label="euler")
plt.legend()
plt.savefig('fig3.png')

finaltime=datetime.now()
print(finaltime-initialtime)
