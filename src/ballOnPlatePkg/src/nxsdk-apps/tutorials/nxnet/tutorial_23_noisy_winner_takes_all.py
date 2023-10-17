
# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2018-2021 Intel Corporation.

# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.

# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

""" Tutorial for noise utilization in spiking winner-takes-all circuits.

The goal of this tutorial is to demonstrate the use of noise injection 
into the membrane current (u) of single-compartment neurons for competition
between similar neurons in a population. For this, we build a network of 
N single compartment neurons forming a winner-takes-all circuit (WTA). In 
this configuration a single neuron, on average, will be firing while inhibiting 
the other, hence its name. The N neurons compete, and the winner takes over
the activity of all other N-1 neurons.

Without noise, a neuron can win because it fired first or because its firing
rate is the highest. In the case of a noisy WTA, as shown here, all neurons 
are allowed to be statistically equivalent. This means the choice of a winner
is random (pseudo-random in practice). When the WTA is part of a bigger system
this means no previous knowledge about the system needs to be encoded on the 
parameters of the WTA. It also avoids overfitting and allows the use of the WTA 
as a primitive for stochastic computations with spiking neurons.

We are going to probe the current and membrane voltage of each neuron in the
network. Probes will also be used to get the spike times for each neuron.

We build and compare two alternative WTA motifs:

1) A single inhibitory neuron mediating the competition between all N neurons.
   Here, the principal neurons connect with excitatory synapses to an auxiliary
   neuron. Depending on its input, the auxiliary neuron sends inhibitory
   feedback to the primary neurons. Size: N+1 neurons and 2N connections.

2) All-to-all lateral inhibition between the N principal neurons. In this case,
   we only use primary neurons which compete through all-to-all lateral
   inhibition. Size: N neurons and N**2 connections.

for each of these motifs a function will be defined, createWtaMotif1 and
createWtaMotif2 respectively.
"""

# Import modules.
import numpy as np
import matplotlib.pyplot as plt  # For graphical displays
import nxsdk.api.n2a as nx      # Nx API
import os
import matplotlib as mpl        # For plotting without GUI
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')


# Define simulation parameters.
N = 5                             # Number of competing neurons
# Used to exclude first neuron per core as a hack for LFSR bug
exclude = 1
runtime = 10000                   # Duration of simulation in time steps


def createWtaMotif1(net, N):
    """Create a winner-takes-all circuit with an auxiliary inhibitory neuron.

    This function creates an NxNet network, which implements a winner-takes-all
    circuit. A single neuron mediates the competition between primary neurons.
    The function uses N2Compiler to compile and initialize the software mappings
    to hardware components and finally 

    Args:
        net: an NxNet object. 
        N: the size of the main population.

    Returns:
        tuple: the current, voltage and spikes probes for principal and auxiliary
               neurons.    
    """
    # Create compartment prototype with noisy current bias.
    prototypePrimary = nx.CompartmentPrototype(biasMant=10,
                                               biasExp=6,
                                               vThMant=100,
                                               logicalCoreId=0,
                                               compartmentVoltageDecay=0,
                                               compartmentCurrentDecay=128,
                                               enableNoise=1,
                                               randomizeCurrent=1,
                                               noiseMantAtCompartment=0,
                                               noiseExpAtCompartment=11,
                                               functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE)

    # Create another prototype with no bias current and no noise injection.
    prototypeAuxiliary = nx.CompartmentPrototype(biasMant=0,
                                                 biasExp=0,
                                                 vThMant=100,
                                                 logicalCoreId=1,
                                                 compartmentVoltageDecay=0,
                                                 compartmentCurrentDecay=4095,
                                                 functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE)

    # Create N Compartments with noisy voltage and driven by a current bias.
    primaryNeurons = net.createCompartmentGroup(
        size=N, prototype=prototypePrimary)
    # Create an auxiliary neuron in resting state.
    auxiliaryNeuron = net.createCompartmentGroup(
        size=1, prototype=prototypeAuxiliary)

    # Create prototypes for excitatory and inhibitory synapses.
    prototypeExc = nx.ConnectionPrototype(
        signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY)
    prototypeInh = nx.ConnectionPrototype(
        signMode=nx.SYNAPSE_SIGN_MODE.INHIBITORY)

    # Define weights for synapses.
    weightsExc = np.array([0]*exclude+[20*4]*(N-exclude))
    weightsInh = np.array([[0]]*exclude+[[-10 * (N-exclude)]]*(N-exclude))

    # On-to-one connect the N neurons to excite the auxiliary neuron
    excitatoryConnections = primaryNeurons.connect(
        auxiliaryNeuron, prototype=prototypeExc, weight=weightsExc)

    # On-to-one connect the auxiliary neuron to exert feedback inhibition on the N neurons
    inhibitoryConnections = auxiliaryNeuron.connect(
        primaryNeurons, prototype=prototypeInh, weight=weightsInh)

    # Create probes to observe neuron's current, voltage and spikes.
    paramsToProbe = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                     nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                     nx.ProbeParameter.SPIKE]

    # Setup voltage, current and spikes probes for all neurons.
    vProbes, uProbes, sProbes = primaryNeurons.probe(paramsToProbe)
    vProbesAux, uProbesAux, sProbesAux = auxiliaryNeuron.probe(paramsToProbe)

    return (vProbes, uProbes, sProbes, vProbesAux, uProbesAux, sProbesAux)


def createWtaMotif2(net, N):
    """ Create a winner-takes-all circuit with synaptic lateral inhibition only.

    This function creates an NxNet network, which implements a winner-takes-all
    circuit through mutual inhibition only. It uses N2Compiler to compile and
    initialize the software mappings to hardware components and finally returns
    the current, voltage and spikes probes for all the neurons.

    Args:
        net: an NxNet object. 
        N: the size of the main population.

    Returns:
        tuple: the current, voltage and spikes probes for all neurons.    
    """
    # Create compartment prototype with noisy current bias.
    prototype = nx.CompartmentPrototype(biasMant=10,
                                        biasExp=6,
                                        vThMant=100,
                                        logicalCoreId=0,
                                        compartmentVoltageDecay=0,
                                        compartmentCurrentDecay=128,
                                        enableNoise=1,
                                        noiseMantAtCompartment=0,
                                        noiseExpAtCompartment=11,
                                        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE)

    # Create N Compartments with noisy voltage and driven by a current bias.
    population = net.createCompartmentGroup(size=N, prototype=prototype)

    # Create prototypes for inhibitory synapses
    connectionPrototypeInh = nx.ConnectionPrototype(
        signMode=nx.SYNAPSE_SIGN_MODE.INHIBITORY, weight=-10)

    # All-to-all connect the N neurons to excite the auxiliary neuron
    inhibitoryConnections = population.connect(
        population, prototype=connectionPrototypeInh)

    # Create probes to observe neuron's current,  voltage and spikes.
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                       nx.ProbeParameter.SPIKE]

    # Setup voltage, current and spikes probes for all neurons.
    (vProbes, uProbes, sProbes) = population.probe(probeParameters)

    return (vProbes, uProbes, sProbes)


if __name__ == "__main__":

    # Create a network object for each motif.
    netWta1 = nx.NxNet()
    netWta2 = nx.NxNet()

    # Setup the wta network.
    probesWta1 = createWtaMotif1(netWta1, N)
    probesWta2 = createWtaMotif2(netWta2, N)

    # Run networks and disconnect.
    netWta1.run(runtime)
    netWta1.disconnect()

    netWta2.run(runtime)
    netWta2.disconnect()

    # Plot current, voltage and spike probes for all compartments of netWta1.
    # Top row plots for principal neurons.
    # Bottom row plots for auxiliary neuron.
    titles = ['Voltage', 'Current', 'Spike Times']
    figWta1 = plt.figure(1)
    figWta1.suptitle('Inhibition with Auxiliary Neuron')
    for n in range(6):
        ax = plt.subplot(2, 3, n+1)
        ax.set_xlim([0, runtime])
        # The following conditional plots all but the first neuron which is fully disconnected (bypassing LFSR bug).
        if n < 3:
            for neuron in range(N-exclude):
                if n == 2:
                    ax = plt.subplot(2*(N-exclude), 3, 3*neuron+3)
                    probesWta1[n][neuron+1].plot()
                    plt.xticks([], [])
                    plt.yticks([], [])
                    ax.set_xlim(0, runtime)
                else:
                    ax = plt.subplot(2, 3, n+1)
                    probesWta1[n][neuron+1].plot()
                    plt.ticklabel_format(
                        style='sci', axis='y', scilimits=(0, 0))
                if neuron < 1:
                    plt.title(titles[n])
        else:
            probesWta1[n].plot()
            plt.title(titles[n-3]+'_aux')
    plt.tight_layout()

    # Plot current, voltage and spike probes for all compartments of netWta2.
    figWta2 = plt.figure(2)
    figWta2.suptitle('Mutual Lateral Inhibition')
    for subplot in range(3):
        plt.subplot(1, 3, subplot+1)
        probesWta2[subplot].plot()
        plt.title(titles[subplot])
        plt.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
    plt.tight_layout()

    # Show plots if display is available else save to file.
    if haveDisplay:
        plt.show()
    else:
        fileName = "%s.png" % os.path.splitext(os.path.basename(__file__))[0]
        print("No display available, saving to file " + fileName + ".")
        figWta1.savefig(fileName)
