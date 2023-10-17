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

""" Tutorial for constructing a neuron's receptive field with mixed synapses.

The goal of this tutorial is to implement a receptive field connectivity which
detects activity from a particular subset of N_t target neurons from a neural 
array of size N. It consists of a detector neuron which receives inhibitory 
synapses from the N_s surrounding neurons of the population to be tested. The
remaining N_t neurons excite the detector neuron. With a sensible choice of
weights, this connectivity implies that the neuron will only be active when the
target neurons are active and the rest are inactive.
"""

# Import modules.
import numpy as np
import matplotlib.pyplot as plt  # For graphical displays
import nxsdk.api.n2a as nx      # Nx API
import os
import matplotlib as mpl        # For plotting without GUI
import matplotlib.patches as mpatches
from nxsdk.utils.plotutils import plotRaster

haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API

# Define simulation parameters.
N = 5                             # Number principal neurons
runtime = 1000                    # Duration of simulation in time steps
width = 0                         # Spread of target neurons 0 picks only the central one
# Range of indexes for target neurons
tNeurons = range(int(N/2)-width, int(N/2)+width+1)
# Template for prototypes of active and inactive neurons
template = np.ones(N, int)
template[tNeurons] = 0             # Set target neurons
w = 110//(1+width*2)              # Weight for excitatory synapses
wInh = -int(w)*(width*2+1)        # Weight for inhibitory synapses

# Matrix of weights from principal neurons to detector
connectionMatrix = wInh*np.ones(N).reshape(1, N)
# Set excitatory weight from target neurons to detector
connectionMatrix[0][tNeurons] = w

# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function defines an example NxNet network, uses N2Compiler to compile
# and initialize the software mappings to hardware components and
# finally returns the probes
# -----------------------------------------------------------------------------


def setupNetwork(net, N):
    """Create a detector neuron whose receptive field targets a neural population.

    This function creates an NxNet network with two compartment groups, one 
    representing the observed population, and the other a single compartment
    detector. The later receives inhibitory and excitatory connections from the 
    observed population. The connectivity defines the detector's receptive field.

    Stochastic spike trains will stimulate each neuron from the observed 
    population. During the first half of the simulation time all neurons will
    receive such stimuli.For the last half of the simulation time Only the target
    neurons will receive stimulation. The detector neuron will then only be 
    active for the last half of the runtime.
    """

    # Create a compartment prototype with no memory of past events.
    prototypeInactive = nx.CompartmentPrototype(vThMant=100,
                                                functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE,
                                                compartmentVoltageDecay=4095,  # Set to maximum value 4095
                                                compartmentCurrentDecay=4095,  # Set to maximum value 4095
                                                )

    # Create another prototype with finite current decay 256, potentially integrating past events.
    prototypeDetector = nx.CompartmentPrototype(vThMant=100,
                                                functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE,
                                                compartmentVoltageDecay=4095,
                                                compartmentCurrentDecay=256,
                                                )

    # Create N Compartments representing the population of principal neurons.
    population = net.createCompartmentGroup(size=N,
                                            prototype=[
                                                prototypeInactive, prototypeInactive],
                                            prototypeMap=template.tolist())

    # Create a compartment for the detector neuron.
    detector = net.createCompartmentGroup(size=1, prototype=prototypeDetector)

    # Create spike generation process to stimulate the principal neurons with stochastic spikes.
    spikes = net.createSpikeGenProcess(N)
    np.random.seed(0)
    spikingProbability = 0.05
    inputSpikes = np.random.rand(N, runtime) < spikingProbability
    spikeTimes = []
    for i in range(N):
        st = np.where(inputSpikes[i, :])[0]
        # Stop spikes at half runtime for non-target neurons.
        if i not in tNeurons:
            st = st[st < int(runtime/2)].tolist()
        else:
            st = st.tolist()
        spikeTimes.append(st)
        spikes.addSpikes(spikeInputPortNodeIds=i, spikeTimes=st)

    # Define a mixed connection prototype to connect population to detector.
    connectionPrototypeMix = nx.ConnectionPrototype(
        signMode=nx.SYNAPSE_SIGN_MODE.MIXED)

    # Define an excitatory connection prototype to connect spike sources to population.
    connectionPrototypeExc = nx.ConnectionPrototype(
        signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY)

    # Connect population to detector with the adequate weight mask.
    excitatoryConnections = population.connect(
        detector, prototype=connectionPrototypeMix, weight=connectionMatrix)

    # Stimulate principal neurons with input spikes.
    stimulate = spikes.connect(
        population, prototype=connectionPrototypeExc, weight=np.eye(N, dtype=int)*110)

    # Create probes to observe neuron's current, voltage and spikes.
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                       nx.ProbeParameter.SPIKE]

    # Setup voltage, current and spikes probes for all neurons.
    probesPopulation = population.probe(probeParameters)
    probesDetector = detector.probe(probeParameters)

    return probesPopulation, probesDetector


if __name__ == "__main__":

    # Create an NxNet network object.
    detectorNet = nx.NxNet()

    # Setup receptive field network.
    probesPop, probesDetect = setupNetwork(detectorNet, N)

    # Run network and free resoruces.
    detectorNet.run(runtime)
    detectorNet.disconnect()

    # Plot current, voltage and spike probes for all compartments.
    titles = ['Current', 'Voltage', 'Spike Times']
    figDetector = plt.figure(1)
    for probe in range(3):
        # Plot probes from principal neurons.
        for neuron in range(N):
            ax1 = plt.subplot(N+1, 3, neuron*3+1+probe)
            pl1 = probesPop[probe][neuron].plot()
            ax1.set_xlim([0, runtime])
            plt.xticks([], [])
            if neuron == 0:
                plt.title(titles[probe])
            if probe == 2 or N > 10:
                plt.yticks([], [])
            if neuron in tNeurons:
                pl1[0].set_color('g')
        # Plot probes from detector neuron.
        ax2 = plt.subplot(N+1, 3, probe+1+N*3)
        ax2.set_xlim([0, runtime])
        pl = probesDetect[probe][0].plot()
        pl[0].set_color('r')
    # Setup legend.
    plt.subplots_adjust(hspace=0.1, wspace=0.3)
    legP = mpatches.Patch(color='blue', label='principal neurons')
    legD = mpatches.Patch(color='red', label='detector neuron')
    legT = mpatches.Patch(color='green', label='target neuron')
    plt.legend(handles=[legP, legD, legT], bbox_to_anchor=(1.3, N+1.5))

    # Show or save plot.
    if haveDisplay:
        plt.show()
    else:
        fileName = "%s.png" % os.path.splitext(os.path.basename(__file__))[0]
        print("No display available, saving to file " + fileName + ".")
        figDetector.savefig(fileName)
