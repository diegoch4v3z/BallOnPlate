"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2018-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express 
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy, 
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are 
expressly stated in the License.
"""

# -----------------------------------------------------------------------------
# Tutorial: tutorial_04_synaptic_delays.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces synaptic delays and will demonstrate how they work
# and how to configure them. We configure four compartments driven by an input
# spike generator. The spike generator is connected to four other compartments
# through four synapses - each with a different synaptic delay.
# We inject three spikes from the spike generator (here at timesteps 5, 15,
# and 30).The first synapse has no synpatic delay, while the second,
# third, and fourth synapses have 12, 28, and 62 timestep delays respectively
# (these numbers are randomly chosen for illustration purposes).
# This tutorial also details the configuration of the balance between the
# synaptic delay precision and the number of dendritic compartments.
#

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

# For plotting without GUI
import matplotlib.pyplot as plt
import nxsdk.api.n2a as nx
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API

# plt is used for graphical displays


# Define a function to setup the network
def setupNetwork(net, synDlys, spikeTimes):

    # -----------------------------------------------------------------------
    # Configure core with 4 compartments driven by input spikes
    # with varying synaptic delays
    # -----------------------------------------------------------------------

    # Configure the dendrite accumulator delay range using numDendriticAccumulators
    # This controls the balance between the delay resolution and the
    # number of dendritic compartments.
    # There are 2**13 dendritic accumulators. numDendriticAccumulators controls the
    # maximum number of delay buckets per compartment and therefore the
    # maximum number of addressable compartments. The number of delay
    # buckets is 2**numDecayExp-2 (due to HW implementation) and the number of
    # addressable compartments is 2**(13-numDecayExp).
    # As you allocate more bits for the delay
    # precision, there are less bits to address the dendritic compartment.
    # For example, here we configure the delayBits to be 6, giving us up
    # to 2**6-2=62 delay buckets per compartment and 2**7=128 compartments.
    # A delayBits value of 0 is interpreted as 1 and 7 is interpreted as
    # 6 (maximum supported). When delayBits is not set the default is 3,
    # thus there are 1024 compartments with 8-2=6 delay buckets.
    # Specify less delay bits in order to increase the number of
    # dendritic compartments and similarly specify more delay bits
    # in order to increase the synaptic delay resolution.
    #
    # Valid sizes for numDendriticAccumulators are [8,16,32,64]

    prototype1 = nx.CompartmentPrototype(vThMant=10,
                                         compartmentVoltageDecay=256,
                                         compartmentCurrentDecay=409,
                                         numDendriticAccumulators=64)

    compartment1 = net.createCompartment(prototype1)
    compartment2 = net.createCompartment(prototype1)
    compartment3 = net.createCompartment(prototype1)
    compartment4 = net.createCompartment(prototype1)

    # Connect compartments 2-5 with compartment1 using varying synaptic delays assigned from synDlys

    connProto1 = nx.ConnectionPrototype(weight=1, delay=synDlys[0],
                                        numDelayBits=6, signMode=2)
    connProto2 = nx.ConnectionPrototype(weight=1, delay=synDlys[1],
                                        numDelayBits=6, signMode=2)
    connProto3 = nx.ConnectionPrototype(weight=1, delay=synDlys[2],
                                        numDelayBits=6, signMode=2)
    connProto4 = nx.ConnectionPrototype(weight=1, delay=synDlys[3],
                                        numDelayBits=6, signMode=2)

    # Create probes for each compartment
    probes = []
    for c in net._compartments:
        probe = c.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                         nx.ProbeParameter.COMPARTMENT_VOLTAGE],
                        probeConditions=None)
        probes.append(probe)

    # Spike Generator
    numPorts = 1
    spikeGen = net.createSpikeGenProcess(numPorts)
    spikeGen.connect(compartment1, prototype=connProto1)
    spikeGen.connect(compartment2, prototype=connProto2)
    spikeGen.connect(compartment3, prototype=connProto3)
    spikeGen.connect(compartment4, prototype=connProto4)

    spikeGen.addSpikes([0], [spikeTimes])

    # Return the configured probes
    return probes


if __name__ == "__main__":

    # Create a network
    net = nx.NxNet()

    # List of varying synaptic delays
    synDlys = (0, 12, 28, 62)

    # List of spike times
    spikeTimes = [5, 15, 30]

    # Setup the network
    probes = setupNetwork(net, synDlys, spikeTimes)

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    net.run(100)
    net.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------
    COMPARTMENT_CURRENT = 0
    COMPARTMENT_VOLTAGE = 1
    uProbes = [probe[COMPARTMENT_CURRENT] for probe in probes]
    vProbes = [probe[COMPARTMENT_VOLTAGE] for probe in probes]

    # The current variable u jumps as the spikes come in for the first
    # synapse. As the delay period elapses for the second, third and fourth
    # synapse the current variable u accumulates and increases.
    # Similarly, the voltage variable v will grow as the synaptic delay
    # periods elapse and will then start to decay.
    fig = plt.figure(1004, figsize=(30, 20))
    k = 1
    numReceiverNeurons = 4
    # For each compartment plot the u and v variables
    for j in range(0, numReceiverNeurons):
        plt.subplot(numReceiverNeurons, 2, k)
        uProbes[j].plot()
        plt.title('u'+str(j))
        k += 1
        plt.subplot(numReceiverNeurons, 2, k)
        vProbes[j].plot()
        plt.title('v'+str(j))
        k += 1

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_04_fig1004.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
