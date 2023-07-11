# Diego Chavez
# Omar Garcia
# New Mexico State University


import nengo
from quadcopterROS import Quadcopter
import gain_sets
import numpy as np


class Model(object):
    def __init__(self, target_func):
        gain_matrix_position, gain_matrix_virtual, gain_matrix_controller, task_to_rotor = gain_sets.get_gain_matrices(gain_set='omar')
        self.model = nengo.Network(label='PID', seed=1)

        with self.model:

            self.copter_node = Quadcopter(target_func=target_func)



            # Obtain data from class Quadcopter
            statesNode = nengo.Node(self.copter_node, size_in=4, size_out=15)       # Node from class Quadcopter
            #self.rotationMatrix = self.copter_node.rotationMatrix()
            statesEnsemble = nengo.Ensemble(n_neurons=1, dimensions =16, neuron_type=nengo.Direct())
            nengo.Connection(statesNode[0:1], statesEnsemble[0:1], synapse=None)
            nengo.Connection(statesNode[2], statesEnsemble[2], synapse=None, transform=0.005)   #Normalizing the state to the max integral value 200
            nengo.Connection(statesNode[3:6], statesEnsemble[3:6], synapse=None)
            nengo.Connection(statesNode[6:], statesEnsemble[7:], synapse=None)

            int_param = 0.2#0.3

            integral_ez = nengo.Ensemble(n_neurons=500, dimensions=1, radius=1)
            nengo.Connection(statesEnsemble[2], integral_ez, synapse=int_param, transform=0.85)#transform 1
            nengo.Connection(integral_ez, integral_ez, synapse=int_param)
            nengo.Connection(integral_ez, statesEnsemble[6], synapse=None)       #Aqui se esta conectando al ensamble porque en el siguiente paso hacemos una multiplicacion matricial. Es decir, queremos que la integral del error en z este guardada en ese espacio. Pero esto no es de donde estamos tomando el Probe.
            denormalizedIntegralEz = nengo.Ensemble(n_neurons=200, radius=200, dimensions=1)
            nengo.Connection(integral_ez, denormalizedIntegralEz, transform=200)






            # PID
            PIDpos = nengo.Ensemble(n_neurons=250, dimensions=3, radius=1.0)
            nengo.Connection(statesEnsemble[0:7], PIDpos, transform=gain_matrix_position, synapse=None)

            # Orientation Error
            taueOri = 0.00001
            # 0.0005 - 8/10, 0.005 - 6/10
            eOri = nengo.Ensemble(n_neurons=600, dimensions=3, radius=0.2)
            nengo.Connection(PIDpos[0:2], eOri[0:2], transform=-1, synapse=taueOri)
            nengo.Connection(statesEnsemble[7:10], eOri[0:3], synapse=taueOri)
            nengo.Connection(statesEnsemble[13], eOri[2], transform=-1, synapse=taueOri)  #Aqui tenia -1

            # Angular Velocity Inertial
            angleVelInerDesired = nengo.Ensemble(n_neurons=500, dimensions=3, radius=0.2)
            nengo.Connection(eOri, angleVelInerDesired, transform=gain_matrix_virtual, synapse=0.000001)

###############################################################################
            # Transforming the Desired Angular velocities from Inertial Frame to Body Frame
            pqrDesired = nengo.Ensemble(n_neurons=300, dimensions=3, radius=0.3)

            # Maxs for relay 1 theta 0.15 0.03

            relay1 = nengo.Ensemble(n_neurons=300, dimensions=3, radius=0.2)
            nengo.Connection(statesEnsemble[8], relay1[0], synapse=None)    #Theta
            nengo.Connection(angleVelInerDesired[0], relay1[1], synapse=None)        #Psi dot desired
            nengo.Connection(angleVelInerDesired[2], relay1[2], synapse=None)        #Phi dot desired

            #Max for relay 2 theta0.15 0.03

            relay2 = nengo.Ensemble(n_neurons=300, dimensions=4, radius=0.2)
            nengo.Connection(statesEnsemble[8], relay2[0], synapse=None)  # Theta
            nengo.Connection(statesEnsemble[7], relay2[1], synapse=None)  # Phi
            nengo.Connection(angleVelInerDesired[1], relay2[2], synapse=None)  # Theta dot desired
            nengo.Connection(angleVelInerDesired[0], relay2[3], synapse=None)  #Psi dot desired

            relay3 = nengo.Ensemble(n_neurons=300, dimensions=4, radius=0.2)
            nengo.Connection(statesEnsemble[8], relay3[0], synapse=None)  # Theta
            nengo.Connection(statesEnsemble[7], relay3[1], synapse=None)  # Phi
            nengo.Connection(angleVelInerDesired[0], relay3[2], synapse=None)        #Psi dot desired
            nengo.Connection(angleVelInerDesired[1], relay3[3], synapse=None)  # Theta dot desired

            def p(x):
                return -np.sin(x[0])*x[1] + x[2]
            def q(x):
                return np.cos(x[0])*np.sin(x[1])*x[3] + np.cos(x[1])*x[2]
            def r(x):
                return np.cos(x[0])*np.cos(x[1])*x[2] - np.sin(x[0])*x[3]

            nengo.Connection(relay1, pqrDesired[0], synapse=0.01, function=p)
            nengo.Connection(relay2, pqrDesired[1], synapse=0.01, function=q)
            nengo.Connection(relay3, pqrDesired[2], synapse=0.01, function=r)

###############################################################################





            full_throttle = nengo.Node([0,0,0,0])

            nengo.Connection(full_throttle, statesNode, synapse=0.001)


            # PROBES
            self.probeStatesNode = nengo.Probe(statesNode)
            self.probeStatesEnsemble = nengo.Probe(statesEnsemble, synapse=0.1)

            self.probe_integral_ez = nengo.Probe(integral_ez, synapse = 0.1)
            self.probe_PIDpos = nengo.Probe(PIDpos, synapse=0.5)
            self.probe_eOri = nengo.Probe(eOri, synapse=0.5)
            self.probe_angleVelInerDes = nengo.Probe(angleVelInerDesired, synapse=0.5)

            self.probe_denormalized_int_ez = nengo.Probe(denormalizedIntegralEz, synapse=0.1)
            self.probe_pqrDesired = nengo.Probe(pqrDesired, synapse=0.5)




    def get_model(self):
        return self.model
    def get_copter(self):
        return self.copter_node
    def get_probe(self):
        return self.probeStatesNode, self.probeStatesEnsemble, self.probe_denormalized_int_ez, self.probe_eOri, self.probe_angleVelInerDes, self.probe_pqrDesired, self.probe_PIDpos #, self.probe_integral_ez, self.probe_PIDpos, self.probe_eOri, self.probe_angleVelIner

