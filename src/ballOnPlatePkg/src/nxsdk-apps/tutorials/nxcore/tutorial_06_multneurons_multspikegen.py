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
# Tutorial: 06_multneurons_multspikegen.py
# -----------------------------------------------------------------------------
# This tutorial configures two neurons/compartments and drives them using
# a spike-train supplied by an external spike generator. The spike generator
# supplies the current independently to the neurons. The membrane  current 'u'
# and the membrane voltage 'v' of both compartments are tracked and plotted.
#
# The spike-generator injects 10 spikes, separated by 10 time-steps, thus
# running on the hardware for a total of 100 time-steps. Plots of u and v
# versus time exhibit intuitively expected spiking behaviour.


# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

from nxsdk.graph.nxinputgen.nxinputgen import BasicSpikeGenerator
from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays

# N2Board module provides access to the neuromorphic hardware
# InputGen module provides the BasicSpikeGenerator used later in the tutorial


# -----------------------------------------------------------------------------
# Function to setup the network
# -----------------------------------------------------------------------------

def setupNetwork():
    """This functions sets up the network, i.e., initializes the N2Board, configures
       neuron compartments, and sets neural connectivity.
       returns: board (N2Board)
    """

    # -----------------------------------------------------------------------------
    # Initialize board
    # -----------------------------------------------------------------------------

    # Board ID
    boardId = 1
    # Number of chips
    numChips = 1
    # Number of cores per chip (a python list with length = numChips)
    numCoresPerChip = [1]
    # Number of synapses per core (a list of lists with length of each
    # element = numCoresPerChip )
    numSynapsesPerCore = [[2]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------------
    # Configure 2 compartments
    # -----------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartments used, are indexed
    # 0 and 1, i.e. cxCfg[0] and cxCfg[1].

    # Current and Voltage decay time-constants are set to 1/16
    # (2^12 factor is for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayU=int(1/16*2**12),
                                     decayV=int(1/16*2**12))

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)

    # The parameter numUpdates controls how many compartment groups will
    # be serviced (4 compartments per group). Specifically, numUpdates = i
    # means that i compartment groups will be update, which translates to
    # compartments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    # Both compartments share the same profile-configs.
    n2Core.cxCfg[0].configure(cxProfile=0, vthProfile=0)
    n2Core.cxCfg[1].configure(cxProfile=0, vthProfile=0)

    # -----------------------------------------------------------------------------
    # Configure synaptic connectivity
    # -----------------------------------------------------------------------------

    # Configure destination compartments for both synapses. In the following,
    # synapse 0 is the connection to compartment CIdx 0 and synapse 1 is the
    # connection to compartment CIdx 1.
    n2Core.synapses[0].CIdx = 0
    n2Core.synapses[1].CIdx = 1

    # Configure synaptic parameters, weight and format ID. Here, for simplicity,
    # we have kept both synaptic weights = 1. "Synapse format ID" refers to the
    # ID of a set of configurations that are common to the synapses using that
    # particular ID. Here, we have only one type of synapses, whose shared properties
    # are encapsulated in synFmtId = 1 (see below).
    n2Core.synapses[0].Wgt = 1
    n2Core.synapses[1].Wgt = 1
    n2Core.synapses[0].synFmtId = 1
    n2Core.synapses[1].synFmtId = 1

    # Configure the synapse format. Here, one may specify additional tweaks to
    # synaptic properties, such as scaling the weights using a weight exponent,
    # or bit-width of the weights, etc. A detailed explanation of these is not
    # within the scope of this tutorial.
    n2Core.synapseFmt[1].wgtExp = 0
    n2Core.synapseFmt[1].wgtBits = 7
    n2Core.synapseFmt[1].numSynapses = 63
    n2Core.synapseFmt[1].cIdxOffset = 0
    n2Core.synapseFmt[1].cIdxMult = 0
    n2Core.synapseFmt[1].idxBits = 1
    n2Core.synapseFmt[1].fanoutType = 2
    n2Core.synapseFmt[1].compression = 0

    # SynapseMap is the data structure that holds mapping from axons to synapses.
    # n2Core.synapseMap[j] corresponds to axonId = j.
    # synapsePtr = index of the first destination synapse
    # synapseLen = number of destination synapses spanned by axonId j (axonal
    #              arbour of axonId j)
    # discreteMapEntry.cxBase pertains to counting the target compartment ID from
    # a base offset = cxBase. This is outside the scope of this tutorial, consult
    # the detailed documentation. But in this particular case, this entry needs to
    # be 0 for both synapseMap entries.

    # Input 1
    n2Core.synapseMap[0].synapsePtr = 0
    n2Core.synapseMap[0].synapseLen = 1
    n2Core.synapseMap[0].discreteMapEntry.cxBase = 0

    # Input 2
    n2Core.synapseMap[1].synapsePtr = 1
    n2Core.synapseMap[1].synapseLen = 1
    n2Core.synapseMap[1].discreteMapEntry.cxBase = 0

    return board


def runNetwork(board, doPlot=True, tFinal=100):

    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------------
    # Configure probes
    # -----------------------------------------------------------------------------

    mon = board.monitor
    # cxState[i] contains the state of cxCfg[i].  The probe method taks a list of
    # compartment indices, i.e. [0] in this example, to obtain the data for the
    # chosen field, e.g. u or v.  The data for each compartment index is returned
    # as an element of a list so in this example the only result is in position 0.
    uProbe = mon.probe(n2Core.cxState, [0, 1], 'u')
    vProbe = mon.probe(n2Core.cxState, [0, 1], 'v')

    # -----------------------------------------------------------------------------
    # Configure Spike Generator
    # -----------------------------------------------------------------------------
    # BasicSpikeGenerator takes the 'board' object as the input and we use 'addSpike'
    # method to inject spikes.
    # BasicSpikeGenerator.addSpike(time, chipId, coreId, axonId)
    #     time: at which spike is injected
    #     chipId: of the target neuro-core
    #     coreId: of the target neuro-core
    #     axonId: of the axon, over which spike is to be injected (same as the one we
    #             used to index synapseMap above)

    # Create basic spike generator object
    bs = BasicSpikeGenerator(board)

    # Add spikes to the object in a loop. Here, we are adding one spike every 10
    # time-steps, from 0 to tFinal.
    for t in range(0, tFinal, 10):
        bs.addSpike(t, 0, 4, 0)
        bs.addSpike(t, 0, 4, 1)

    # -----------------------------------------------------------------------------
    # Run
    # -----------------------------------------------------------------------------

    board.run(tFinal)

    # -----------------------------------------------------------------------------
    # Plot
    # -----------------------------------------------------------------------------
    # Membrane voltages v increase exponentially due to spike injection, for both
    # compartments. Upon reaching the threshold of 640, the compartments spike and
    # reset. Since there is no refractory period, the voltages immediately begin to
    # rise again.
    if doPlot:
        fig1 = plt.figure(10)
        plt.subplot(2, 1, 1)
        inputTime = [j for j in range(tFinal)]
        inputSpikes = [0 for j in range(tFinal)]
        for j, spk in enumerate(inputSpikes):
            if j % 10 == 0 and j > 0:
                inputSpikes[j] = 64
        plt.plot(inputTime, inputSpikes)
        plt.title('Input spike train')
        plt.xlabel('Time')
        plt.ylabel('Driving current')

        fig2 = plt.figure(11)
        plt.subplot(2, 1, 1)
        uProbe[0].plot()
        plt.title('First Neuron')
        plt.ylabel('Membrane current')
        plt.subplot(2, 1, 2)
        vProbe[0].plot()
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        fig3 = plt.figure(12)
        plt.subplot(2, 1, 1)
        uProbe[1].plot()
        plt.title('Second Neuron')
        plt.ylabel('Membrane current')
        plt.subplot(2, 1, 2)
        vProbe[1].plot()
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        if haveDisplay:
            plt.show()
        else:
            fileName1 = "tutorial_06_fig10.png"
            fileName2 = "tutorial_06_fig11.png"
            fileName3 = "tutorial_06_fig12.png"
            print("No display available, saving to files " +
                  fileName1 + ", " + fileName2 + " and " + fileName3 + ".")
            fig1.savefig(fileName1)
            fig2.savefig(fileName2)
            fig3.savefig(fileName3)
    return 0


if __name__ == '__main__':

    # --------------------------------------------------------------------------
    # Setup the network
    # --------------------------------------------------------------------------
    board = setupNetwork()

    # --------------------------------------------------------------------------
    # Run the network and plot results
    # --------------------------------------------------------------------------
    runNetwork(board)
    board.disconnect()
