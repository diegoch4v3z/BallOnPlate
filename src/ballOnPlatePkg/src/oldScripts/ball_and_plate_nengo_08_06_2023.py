
#This code simulates the dynamics of the ball and plate with a feedback state
#control using spiking neural networks

import matplotlib.pyplot as plt
#%matplotlib inline

import nengo
import numpy as nu
#import nengo_loihi

#Declaration of gravitational acceleration (g), control gains (kp_x,kd_x,kp_y,kd_y)
#and synaptic delay (t_synapse)

g = 9.81
kp_x = -1.1417
kd_x = -0.5708
kp_y = -1.1417
kd_y = -0.5708
t_synapse = 0.1

with nengo.Network(label="ballandplate") as model:
    
#In order to model the system of the form dot{x} = Ax + Bu , this one is transformed to 
# dot{x}= 1/T(f'+g'-x) with f'=TAx + x and g'= TBu  

#Two ensembles are created to represent the matrices A and B
    
    F = nengo.Ensemble(n_neurons=400, dimensions=4)
    G = nengo.Ensemble(n_neurons=200, dimensions=2)
    
#The functions A_fun and B_fun model the terms f'=TAx and g'=TBu. 

    def f_fun(x):
        return [x[1]*t_synapse,0,x[3]*t_synapse,0]+x
    def g_fun(x):
        return [0,-nu.sin(x[0])*5/7*g*t_synapse,0,-nu.sin(x[1])*5/7*g*t_synapse]
    
    nengo.Connection(G,F,synapse = t_synapse,function=g_fun)
    nengo.Connection(F,F,synapse = t_synapse,function=f_fun)

#The signals stim_ref_x and stim_ref_y are the position references that be encoded through
#the ensemble ref
    
    #stim_ref = nengo.Node(lambda t: [0.2*nu.sin(0.5*t),0.3*nu.cos(0.5*t)])
    stim_ref = nengo.Node([0.2,0.2])
    
    ref = nengo.Ensemble(n_neurons=200,dimensions=2)
    
    nengo.Connection(stim_ref,ref)

#The ensemble msrmt encodes the position of the ball that is sent to the error ensemble
#applying a transformation of -1. The error ensemble also receives the references signals to calculate
#the position error.
    
    msrmt = nengo.Ensemble(n_neurons=200,dimensions=2)
    error = nengo.Ensemble(n_neurons=400,dimensions=2)
    
    nengo.Connection(ref,error)
    nengo.Connection(F[[0,2]],msrmt)
    nengo.Connection(msrmt,error,transform=-1)

#The error derivatives are calculated using two ensembles where the signals are
#sending with two different synaptic delays and applying a transformation inversely
#proportional to the difference between the synaptic delays
    
    d_e_x = nengo.Ensemble(n_neurons=100, dimensions = 1)
    nengo.Connection(error[0],d_e_x,synapse=0.05,transform=20)
    nengo.Connection(error[0],d_e_x,synapse=0.1,transform=-20)
    
    d_e_y = nengo.Ensemble(n_neurons=100,dimensions=1)
    nengo.Connection(error[1],d_e_y,synapse=0.05,transform=20)
    nengo.Connection(error[1],d_e_y,synapse=0.1,transform=-20)
    
#The error and their derivatives are sent to the ensemble B applying the 
#transform corresponding to the control gains to generate the control signals
  
    def control(x):
        return [x[0]*kp_x,x[1]*kp_y]    
    
    nengo.Connection(error,G,function=control)        

    def de_x(x):
        return [x,0]
    def de_y(x):
        return [0,x]

    nengo.Connection(d_e_x,G,function=de_x,transform=kd_x)
    nengo.Connection(d_e_y,G,function=de_y,transform=kd_x)
    
    ens_probe = nengo.Probe(F,synapse=0.1)
    ref_probe = nengo.Probe(ref,synapse=0.1)
    
    
    
    
    