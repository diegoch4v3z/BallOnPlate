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
# Tutorial: tutorial_11_box_synapse.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces a box synapse. Instead of an exponentially
# decaying post synaptic response after receiving a spike, we want to
# have a box response, i.e. a constant post synaptic current for a
# specified amount of time. To achieve this we set the decayU to zero,
# add the weight at t and subtract the weight at t+dly. We will
# reuse the synapses[i].dly as the box length instead of the synaptic delay.
# In order to illustrate the box synapse we configure two synapses,
# one as a default synapse with a synaptic delay of 7 (please refer to
# the synaptic delays tutorial for more details) and one as a box
# synapse with a box length of 7.
#

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

import nxsdk.api.n2a as nx
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays
# Nx API

# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function defines an example NxNet network, uses N2Compiler to compile
# and initialize the software mappings to hardware components and
# finally returns the board
# -----------------------------------------------------------------------------


def setupNetwork(net):
    # -----------------------------------------------------------------------
    # Configure core with single neuron and two synapses driven by
    # a single input spike
    # -----------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartments that are used
    # are also indices 0, 1, 2 i.e. cxCfg[0],...,cxCfg[2].
    # The compartments will share the same profile.

    # The current decay (decayU) is set to 0 to highlight the box synapse behavior.
    prototype1 = nx.CompartmentPrototype(
        vThMant=10,
        functionalState=2,
        logicalCoreId=0,
        compartmentCurrentDecay=0,
        numDendriticAccumulators=64
    )

    compartment1 = net.createCompartment(prototype1)
    compartment2 = net.createCompartment(prototype1)

    # Create Connection Prototype one with box synapse one without
    # Setting postSynResponseMode to 1 creates a box synapse
    connProto1 = nx.ConnectionPrototype(weight=1, delay=7,
                                        numWeightBits=-1, numDelayBits=6,
                                        disableDelay=0, compressionMode=0,
                                        signMode=2)

    connProto2 = nx.ConnectionPrototype(weight=1, delay=7,
                                        numWeightBits=-1, numDelayBits=6,
                                        disableDelay=0, compressionMode=0,
                                        signMode=2, postSynResponseMode=1)

    si1 = net.createSpikeGenProcess(1)
    si2 = net.createSpikeGenProcess(1)

    conn11 = si1.connect(compartment1, connProto1)
    conn22 = si2.connect(compartment2, connProto2)

    si1.addSpikes(0, [3])
    si2.addSpikes(0, [3])

    # Create Probes
    # Creating probes to observer Current and Voltage for all 2 compartments
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT]
    cxProbe1 = compartment1.probe(probeParameters)
    cxProbe2 = compartment2.probe(probeParameters)

    return (cxProbe1, cxProbe2)


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # Create a network
    net = nx.NxNet()

    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------

    probes = setupNetwork(net)

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------
    net.run(100)
    net.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    # The neuron is driven by a single spike inserted at time step 3.
    # Each synapse illustrates the manifestation of this spike depending
    # on the synapse type.
    # The spike arrives at time step 3, but with a synaptic delay of 7
    # u does not accumulate until after the synaptic delay. u remains
    # constant after accumulating due to the decayU = 0.
    cx1Probe, cx2Probe = probes

    fig = plt.figure(10)
    plt.subplot(2, 1, 1)
    cx1Probe[0].plot()
    plt.title('Synapse with synaptic delay')
    plt.ylabel('Membrane current')

    # The spike arrives at time step 3 when the weight is immediately
    # accumulated in u. After the specified time (7 in this tutorial)
    # the weight is subtracted bringing u back to 0.
    plt.subplot(2, 1, 2)
    cx2Probe[0].plot()
    plt.title('Box synapse')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_11_fig10.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
