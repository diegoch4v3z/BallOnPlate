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

# ------------------------------------------------------------------------------
# Tutorial: tutorial_18b_delay_learning.py
# ------------------------------------------------------------------------------
# This tutorial illustrates the use of learning rules. Two input axons (axon 0
# and axon 1) are connected via independent synapses to same compartment 0.
# Axon 0 is the driver for the compartment and axon 1 is the learning enabled
# axon whose pre-synaptic traces (X) are inputs into the learning rule.
# Post synaptic traces (Y) from compartment 0 back to synapse 1 are configured
# as well. Various learning rules are configured along with various spike
# timings to illustrate the affects of learning. An illustration of the setup
# is shown below:
#                                                            ______________
# axon 0 -------> synapse 0 (learning disabled) --------->  |              |
#                                                           |compartment 0 |
# axon 1 -------> synapse 1 (learning enabled)  --------->  |______________|
#        ------->                               <---------
#      pre-traces (X)                         post-traces (Y)
#
# This example illustrates the use of a delay learning rule.

# ------------------------------------------------------------------------------
# Import modules
# ------------------------------------------------------------------------------
from nxsdk.graph.nxinputgen.nxinputgen import BasicSpikeGenerator
import nxsdk.compiler.microcodegen.interface as uci
from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import math
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# ------------------------------------------------------------------------------
# Function: setupNetwork
# This function sets up the network and returns the board
# ------------------------------------------------------------------------------


def setupNetwork():
    # --------------------------------------------------------------------------
    # Configure learning example
    # --------------------------------------------------------------------------
    learningRule = 'dd = -2^-4*d*y0'
    spikeTimes0 = [5, 30, 45]
    spikeTimes1 = [15, 40, 60]
    synDly = 16
    maxDly = 32
    tEpoch = 16

    # --------------------------------------------------------------------------
    # Initialize board
    # --------------------------------------------------------------------------
    # N2Board ID
    boardId = 1
    # Number of chips
    numChips = 1
    # Number of cores per chip
    numCoresPerChip = [1]
    # Number of synapses per core
    numSynapsesPerCore = [[4]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # --------------------------------------------------------------------------
    # Configure 1 compartment (compartment 0) that spikes to provide
    # post synaptic traces (Y) for use by the learning enabled synapse, i.e.
    # synapse 1.
    # --------------------------------------------------------------------------

    # Configure numUpdates register. numUpdates sets the number of
    # compartment groups to update. numStdp sets the number of
    # learning-enabled axons, that is the number of independent axons that
    # have learning-enabled synapses.
    n2Core.numUpdates.configure(numUpdates=1, numStdp=1)

    # Set fast decay rates for U and V.
    # bapAction = 2 disables refactory state (bit 0) and enables bAP (bit 1)
    # bapSrc = 0 activates bAP when compartment 0 voltage v crosses threshold
    n2Core.cxProfileCfg[0].configure(decayU=int(1/1.25*2**12),
                                     decayV=int(1/1.25*2**12),
                                     bapAction=2,
                                     bapSrc=0)

    # Threshold (vth) and synaptic weights are chosen so that spikes entering
    # input axon 0 result in compartment 0 spikes, while spikes entering input
    # axon 1 do not result in compartment 0 spikes.
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^6) = 150*(2^6) = 9600.
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=150)

    # --------------------------------------------------------------------------
    # Configure pre-synaptic traces
    # See tutorial 05 for a detailed description of pre-synaptic traces.  In
    # this illustration, the following configuration is used:
    #
    # synMapId axonNumber entryType          historicTraces
    # 0        0          DiscreteMapEntry   N/A
    # 1        1          DiscreteMapEntry
    # 2                     SingleTraceEntry x1e0, x1e1
    # 3                     SingleTraceEntry x1e0, x1e1
    # --------------------------------------------------------------------------
    # Create axon 0
    n2Core.synapseMap[0].synapsePtr = 0
    n2Core.synapseMap[0].synapseLen = 1
    n2Core.synapseMap[0].discreteMapEntry.configure()

    # Create axon 1
    n2Core.synapseMap[2].synapsePtr = 1
    n2Core.synapseMap[2].synapseLen = 1
    n2Core.synapseMap[2].discreteMapEntry.configure()
    # preProfile selects one of 16 registers in stdpPreProfileCfg
    # tcs selects one of the 3 registers in stdpPreCfg
    numTraces = 0
    numTraceHist = math.ceil(maxDly / tEpoch)
    numTraceEntries = int(math.ceil((numTraces+1)*(numTraceHist+1)/2))
    for i in range(numTraceEntries):
        n2Core.synapseMap[3+i].singleTraceEntry.configure(preProfile=0,
                                                          tcs=0)

    # Configure pre trace behavior (trace decay time constant and the level
    # by which a trace gets incremented upon each spike). A helper class
    # (TraceCfgGen) is available to generate the register configuration based
    # on the time constant (tau) and increment (spikeLevel (integer and
    # fractional parts)) and writes it to the appropriate registers.
    tcg = TraceCfgGen()
    tc = tcg.genTraceCfg(tau=10,
                         spikeLevelInt=40,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.stdpPreCfg[0])

    # Configure what type of TraceEntry is used per axon (numTraces = 0 is for
    # single trace entry and numTraces = 1 is for two traces, i.e. selecting
    # between the corresponding synapse map trace entry type), how many
    # historic epochs there are per axon (numTraceHist = 0 maps to
    # numTraceHist+1 = 1 trace histories) and when to update a pre trace
    # (updateAlways updates a trace every epoch irrespective whether there was
    # an input spike or whether the pre trace was non-zero). stdpProfile
    # is described when stdpProfileCfg is configured.
    n2Core.stdpPreProfileCfg[0].configure(updateAlways=1,
                                          numTraces=numTraces,
                                          numTraceHist=numTraceHist,
                                          stdpProfile=0)

    # --------------------------------------------------------------------------
    # Configure post-synaptic traces
    #
    # --------------------------------------------------------------------------
    # traceProfile
    #   * Maps 4 post-synaptic trace configurations (stdpPostCfg) to three
    #     traces y1, y2, and y3.
    #   * It has modes 0 through 3. Mode 3 maps stdpPostCfg[0] to y1,
    #     stdpPostCfg[1] to y2, and stdpPostCfg[2] to y3
    # stdpProfile is described when stdpProfileCfg is configured.
    n2Core.stdpPostState[0].configure(traceProfile=3,
                                      stdpProfile=0)

    # stdpPostCfg is configured in the same way as the stdpPreCfg
    tcg2 = TraceCfgGen()
    tc2 = tcg2.genTraceCfg(tau=10,
                           spikeLevelInt=40,
                           spikeLevelFrac=0)
    tc2.writeToRegister(n2Core.stdpPostCfg[0])

    tcg3 = TraceCfgGen()
    tc3 = tcg3.genTraceCfg(tau=6,  # use a faster decay rate than first cfg
                           spikeLevelInt=40,
                           spikeLevelFrac=0)
    tc3.writeToRegister(n2Core.stdpPostCfg[1])

    # --------------------------------------------------------------------------
    # Configure global learning parameters
    # --------------------------------------------------------------------------
    # Configure global STDP configurations, e.g. at what synMapId the first
    # learning enabled axon starts, and also how many reward axons are
    # configured (reward axons are not used in this tutorial)
    n2Core.stdpCfg[0].firstLearningIndex = 2

    # Configure the learning epoch, that is, at what interval to update
    # synaptic traces (as well as synaptic state)
    n2Core.timeState[0].tepoch = tEpoch

    # --------------------------------------------------------------------------
    # Configure 2 synapses, one for each input axons.
    # --------------------------------------------------------------------------
    # Configure synapse for input axon 0
    # The first synapse has a large weight designed to cause compartment 0 to
    # spike. Learning is disabled on this synapse.
    n2Core.synapses[0].configure(CIdx=0,
                                 Wgt=200,
                                 Dly=0,
                                 synFmtId=1)

    # Configure synapse for axon 1
    # Learning is enabled and it uses a lower weight.
    n2Core.synapses[1].configure(CIdx=0,
                                 Wgt=50,
                                 Dly=synDly,
                                 synFmtId=2,
                                 LrnEn=1)

    # Configure synapseFmt 1 (learning disabled)
    n2Core.synapseFmt[1].wgtBits = 7     # 8 bits per wgt
    n2Core.synapseFmt[1].numSynapses = 1  # Number of synapses per synEntry
    n2Core.synapseFmt[1].idxBits = 1     # 6 bits for cIdx
    n2Core.synapseFmt[1].compression = 3  # Dense compression
    n2Core.synapseFmt[1].fanoutType = 2  # Excitatory synapse

    # Configure synapseFmt 2 (learning enabled)
    n2Core.synapseFmt[2].wgtBits = 7     # 8 bits per wgt
    n2Core.synapseFmt[2].numSynapses = 1  # Number of synapses per synEntry
    n2Core.synapseFmt[2].idxBits = 1     # 6 bits for cIdx
    n2Core.synapseFmt[2].compression = 3  # Dense compression
    n2Core.synapseFmt[2].fanoutType = 2  # Excitatory synapse
    n2Core.synapseFmt[2].stdpProfile = 0
    n2Core.synapseFmt[2].learningCfg = 1  # Enable learning for all synapses

    # --------------------------------------------------------------------------
    # Configure learning rule
    # The stdpProfileCfg index used is specified by the sum of the stdpProfile
    # settings. Specifically, stdpProfile is set in stdpPreProfileCfg,
    # synapseFmt, and stdpPostState.  In this tutorial, stdpProfile = 0 in all
    # three settings, as such, the stdpProfileCfg index used is 0.
    # --------------------------------------------------------------------------
    # The ruleToCode utility generates the required uCode.
    uc = uci.ruleToUCode([learningRule], False)
    for ct in range(0, uc.numUCodes):
        n2Core.stdpUcodeMem[ct].word = uc.uCodes[ct]
    # uCodePtr points to the learning rule in the stdpUCodeMem.
    n2Core.stdpProfileCfg[0].configure(uCodePtr=0,
                                       requireY=1,
                                       decimateExp=uc.decimateExponent,
                                       numProducts=uc.numProducts)

    # --------------------------------------------------------------------------
    # Configure spike generator to inject spikes into input axon 0 and axon 1
    # Recall, spikes have to to be directed to the mapEntry in synapseMap and
    # not to the pre trace entries.
    # --------------------------------------------------------------------------
    sg = BasicSpikeGenerator(board)
    axIds = [0, 2]
    for axId in axIds:
        if axId == 0:
            for ct in range(0, len(spikeTimes0)):
                sg.addSpike(time=spikeTimes0[ct], chipId=0, coreId=4,
                            axonId=axId)
        else:
            for ct in range(0, len(spikeTimes1)):
                sg.addSpike(time=spikeTimes1[ct], chipId=0, coreId=4,
                            axonId=axId)

    return board


def runNetwork(board, doPlot):
    # Retrieve the only core we need for experiment
    n2Core = board.n2Chips[0].n2Cores[0]

    # --------------------------------------------------------------------------
    # Configure probes for axon 1 pre trace and compartment 0 current (u),
    # voltage (v), and spikes
    # --------------------------------------------------------------------------
    mon = board.monitor
    ax2x1Probes = [mon.probe(n2Core.synapseMap, [3], 'x1e0')[0],
                   mon.probe(n2Core.synapseMap, [3], 'x1e1')[0]]
    uProbes = mon.probe(n2Core.cxState, [0], 'u')
    vProbes = mon.probe(n2Core.cxState, [0], 'v')
    spikeProbes = mon.probe(n2Core.cxState, [0], 'spike')

    wgtProbe = mon.probe(n2Core.synapses, [1], 'Wgt')
    dlyProbe = mon.probe(n2Core.synapses, [1], 'Dly')

    # --------------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------------
    board.run(100)

    if doPlot:
        fig = plt.figure(1)
        plt.subplot(6, 1, 1)
        uProbes[0].plot()
        plt.title('u0')
        plt.subplot(6, 1, 2)
        vProbes[0].plot()
        plt.title('v0')
        plt.subplot(6, 1, 3)
        spikeProbes[0].plot()
        plt.title('spikes 0')
        plt.subplot(6, 1, 4)
        ax2x1Probes[0].plot()
        plt.title('x1e0')

        plt.subplot(6, 1, 5)
        wgtProbe[0].plot()
        plt.title('Weight of Synapse[1]')
        plt.subplot(6, 1, 6)
        dlyProbe[0].plot()
        plt.title('Delay of Synapse[1]')

        if haveDisplay:
            plt.show()
        else:
            fileName1 = "tutorial_18_fig1.png"
            print("No display available, saving to file " + fileName1 + ".")
            fig.savefig(fileName1)

    return uProbes, wgtProbe, dlyProbe


if __name__ == "__main__":
    # --------------------------------------------------------------------------
    # Setup the network
    # --------------------------------------------------------------------------

    # Four examples are available: estdpPos, estdpNeg, otherPos, otherNeg
    board = setupNetwork()

    # --------------------------------------------------------------------------
    # Run the network and plot results
    # --------------------------------------------------------------------------
    plotResult = True
    runNetwork(board, plotResult)
    board.disconnect()
