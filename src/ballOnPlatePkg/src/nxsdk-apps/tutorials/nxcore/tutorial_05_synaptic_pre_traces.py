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
# Tutorial: tutorial_05_synaptic_pre_traces.py
# ------------------------------------------------------------------------------
# The purpose of this tutorial is to demonstrate the configuration and usage
# of synaptic pre traces. Pre traces are associated with input axons,
# i.e. with entries in the synapseMap register. An input axon can have a
# single (x1) or two synaptic pre traces (x1, x2) associated with it, whose
# state is available to all synapses of that axon. Whereas the connectivity
# information of an input axon is encoded via a discreteMap entry, the pre
# trace information is encoded into a SingleTraceEntry or a TwoTraceEntry.
# SingleTraceEntries encode the trace state of two historic epochs for a
# single trace (x1) while TwoTraceEntries encode the trace state of a single
# historic epoch for two independent traces (x1, x2).
# If any learning-enabled synapse of an axon has a synaptic delay > 0,
# then multiple historic epochs are required to store the pre trace state
# from prior learning epochs in order to compute the particular
# synapse-specific pre trace value for a synapse with delay if an input spike
# arrived in a prior learning epoch.
# To demonstrate the configuration of synaptic pre traces via
# singleTraceEntries and twoTraceEntries with and without synaptic delay,
# we use 4 different compartments that receive input via 4 input input axons
# using both singleTraceEntries and twoTraceEntries to encode pre traces.
# We then inject the same spike sequence into each input axon and discuss the
#  the behavior of the synaptic pre traces.

# ------------------------------------------------------------------------------
# Import modules
# ------------------------------------------------------------------------------
from nxsdk.graph.nxinputgen.nxinputgen import BasicSpikeGenerator
from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
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
    # Configure 4 compartments that we solely use as detectors for input spike
    # arriving via 4 input axons.
    # --------------------------------------------------------------------------
    # Configure numUpdates register. numUpdates sets the number of
    # compartment groups to update. numStdp sets the number of
    # learning-enabled axons, that is the number of independent axons that
    # have learning-enabled synapses.
    n2Core.numUpdates.configure(numUpdates=1, numStdp=4)
    n2Core.cxProfileCfg[0].configure(decayU=int(1/5*2**12))
    # We will  use a large threshold because we will only monitor u and don't
    # need any of the compartments to spike.
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=1000)

    # --------------------------------------------------------------------------
    # Configure 4 input axons that each connect to one of the detector
    # compartments.
    # Synaptic traces (as well as synaptic state) are updated in regular
    # learning epochs of duration tEpoch (here we use tEpoch = 2). Depending
    # on the maximum delay of an axon's synapses, a different number of
    # historic traces are required to support learning with synaptic delays.
    # Depending on the type of pre traces, a different number of pre trace
    # entries are required to store the required number of historic traces.
    # SingleTraceEntries offer only a single independent trace (x1) but two
    # historic trace values per entry (x1e0, x1e1), whereas TwoTraceEntries
    # offer two independent traces (x1, x2) but only one historic trace
    # value per entry (x1e0, x2e0).
    # The number of historic traces is:
    #     numTraceHist = ceil(maxDly/tEpoch)+1
    # where maxDly is the maximum synaptic delay of an axon. The number of
    # trace entries is:
    #     numTraceEntries = ceil((numTraces+1)(numTraceHist)/2)
    # where numTraces is 0 for SingleTraceEntries and 1 for TwoTraceEntries.
    # The 4 axons we configure have the following features to illustrate the
    # all aspects of the two types of trace entries:
    #    TraceEntryType    dstCompartment synDly numTraceHist numTraceEntries
    # 1. singleTraceEntry  0              0      1            1
    # 2. twoTraceEntry     1              0      1            1
    # 3. singleTraceEntry  2              3      3            2
    # 4. twoTraceEntry     3              3      3            3
    # Because of the different number of trace entries per axon,
    # the synapseMap structure will look like this:
    # synMapId axonNumber entryType          historicTraces
    # 0        1          DiscreteMapEntry
    # 1                     SingleTraceEntry x1e0, x1e1
    # 2        2          DiscreteMapEntry
    # 3                     TwoTraceEntry    x1e0, x2e0
    # 4        3          DiscreteMapEntry
    # 5                     SingleTraceEntry x1e0, x1e1
    # 6                     SingleTraceEntry x1e2, x1e3
    # 7        4          DiscreteMapEntry
    # 8                     TwoTraceEntry    x1e0, x2e0
    # 9                     TwoTraceEntry    x1e1, x2e1
    # 10                    TwoTraceEntry    x1e2, x2e2
    # For axons 1 and 2, we create two synapses without delay and for axons 3
    # and 4 we create two synapses with dly=3.
    # Irrespective of how many traceEntries there are per axon, only the
    # first traceEntry following the mapEntry, only the first one specify the
    # preProfile and tcs* fields. The preProfile field selects a shared
    # entry in the StdpPreProfileCfg register and the tcs* fields select
    # 1 or the 3 trace configuration in the StdpPreCfg register to use for a
    # trace. A SingleTraceEntry has only one trace and thus only one tcs
    # field while a TwoTraceEntry has two traces and thus a tc1 and tcs2 field.
    # --------------------------------------------------------------------------
    # Create axon 1
    n2Core.synapseMap[0].synapsePtr = 0
    n2Core.synapseMap[0].synapseLen = 1
    n2Core.synapseMap[0].discreteMapEntry.configure()
    n2Core.synapseMap[1].singleTraceEntry.configure(preProfile=0,
                                                    tcs=0)
    # Create axon 2
    n2Core.synapseMap[2].synapsePtr = 1
    n2Core.synapseMap[2].synapseLen = 1
    n2Core.synapseMap[2].discreteMapEntry.configure()
    n2Core.synapseMap[3].twoTraceEntry.configure(preProfile=1,
                                                 tcs1=1,
                                                 tcs2=2)
    # Create axon 3
    n2Core.synapseMap[4].synapsePtr = 2
    n2Core.synapseMap[4].synapseLen = 1
    n2Core.synapseMap[4].discreteMapEntry.configure()
    n2Core.synapseMap[5].singleTraceEntry.configure(preProfile=2,
                                                    tcs=0)
    n2Core.synapseMap[6].singleTraceEntry.configure()
    # Create axon 4
    n2Core.synapseMap[7].synapsePtr = 3
    n2Core.synapseMap[7].synapseLen = 1
    n2Core.synapseMap[7].discreteMapEntry.configure()
    n2Core.synapseMap[8].twoTraceEntry.configure(preProfile=3,
                                                 tcs1=1,
                                                 tcs2=2)
    n2Core.synapseMap[9].twoTraceEntry.configure()
    n2Core.synapseMap[10].twoTraceEntry.configure()

    # Configure pre trace behavior (trace decay time constant and the level
    # by which a trace gets incremented upon each spike). Since the register
    # encoding of this information is quite complex, we use a helper class
    # (TraceCfgGen) that generates the register configuration based on the time
    # constant (tau) and increment (spikeLevel) and writes it to the appropriate
    # registers. The tcs* fields in a SingleTraceEntry or TwoTraceEntry
    # reference 1 of the 3 possible trace configurations.
    tcg = TraceCfgGen()
    tc = tcg.genTraceCfg(tau=15,
                         spikeLevelInt=40,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.stdpPreCfg[0])
    tc = tcg.genTraceCfg(tau=30,
                         spikeLevelInt=20,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.stdpPreCfg[1])
    tc = tcg.genTraceCfg(tau=45,
                         spikeLevelInt=10,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.stdpPreCfg[2])

    # Configure what type of TraceEntry is used per axon (numTraces),
    # how many historic epochs there are per axon
    # (numTraceHist = ceil(maxDly/tEpoch)) and when to update a pre trace
    # (updateAlways updates a trace every epoch irrespective whether there was
    # an input spike or whether the pre trace was non-zero).
    n2Core.stdpPreProfileCfg[0].configure(updateAlways=1,
                                          numTraces=0,
                                          numTraceHist=0)
    n2Core.stdpPreProfileCfg[1].configure(updateAlways=1,
                                          numTraces=1,
                                          numTraceHist=0)
    n2Core.stdpPreProfileCfg[2].configure(updateAlways=1,
                                          numTraces=0,
                                          numTraceHist=2)
    n2Core.stdpPreProfileCfg[3].configure(updateAlways=1,
                                          numTraces=1,
                                          numTraceHist=2)

    # Configure at what synMapId the first learning enabled axon starts
    n2Core.stdpCfg[0].firstLearningIndex = 0

    # Configure the learning epoch, that is, at what interval to update
    # synaptic traces (as well as synaptic state)
    n2Core.timeState[0].tepoch = 2

    # --------------------------------------------------------------------------
    # Configure 4 synapses for the 4 input axons. 2 without delay and 2 with
    # delay. For simplicity, we use the same synapseFmt for all synapses.
    # --------------------------------------------------------------------------
    # Configure synapse for axon 1
    n2Core.synapses[0].configure(CIdx=0,
                                 Wgt=1,
                                 Dly=0,
                                 synFmtId=1)
    # Configure synapse for axon 2
    n2Core.synapses[1].configure(CIdx=1,
                                 Wgt=1,
                                 Dly=0,
                                 synFmtId=1)
    # Configure synapse for axon 3
    n2Core.synapses[2].configure(CIdx=2,
                                 Wgt=1,
                                 Dly=3,
                                 synFmtId=1)
    # Configure synapse for axon 4
    n2Core.synapses[3].configure(CIdx=3,
                                 Wgt=1,
                                 Dly=3,
                                 synFmtId=1)

    # Configure synapseFmt
    n2Core.synapseFmt[1].wgtBits = 7     # 8 bits per wgt
    n2Core.synapseFmt[1].dlyBits = 2     # 2 bits per dly
    n2Core.synapseFmt[1].numSynapses = 1  # Number of synapses per synEntry
    n2Core.synapseFmt[1].idxBits = 1     # 6 bits for cIdx
    n2Core.synapseFmt[1].compression = 3  # Sparse compression
    n2Core.synapseFmt[1].fanoutType = 2  # Excitatory synapse

    # --------------------------------------------------------------------------
    # Configure spike generator to inject spikes into all 4 axons at the same
    #  time. Spikes have to to be directed to the mapEntry in synapseMap and
    # not to any of the pre trace entries. If spikes are directed to pre
    # trace entries, those entries will be interpreted as mapEntries,
    # most likely leading to incorrect synapseMem lookups.
    # --------------------------------------------------------------------------
    sg = BasicSpikeGenerator(board)
    spikeTimes = [5, 16, 50]
    axIds = [0, 2, 4, 7]
    for t in spikeTimes:
        for axId in axIds:
            sg.addSpike(time=t, chipId=0, coreId=4, axonId=axId)

    return board


def runNetwork(board, doPlot):
    # Retrieve the only core we need for experiment
    n2Core = board.n2Chips[0].n2Cores[0]

    # --------------------------------------------------------------------------
    # Configure separate probe lists for each type of pre trace and the 'u'
    # variable of the detector compartments.
    # --------------------------------------------------------------------------
    mon = board.monitor
    ax1x1Probes = [mon.probe(n2Core.synapseMap, [1],   'x1e0')[0],
                   mon.probe(n2Core.synapseMap, [1], 'x1e1')[0]]
    ax2x1Probes = mon.probe(n2Core.synapseMap, [3], 'x1')
    ax2x2Probes = mon.probe(n2Core.synapseMap, [3], 'x2')
    ax3x1Probes = [mon.probe(n2Core.synapseMap, [5], 'x1e0')[0],
                   mon.probe(n2Core.synapseMap, [5], 'x1e1')[0],
                   mon.probe(n2Core.synapseMap, [6], 'x1e0')[0],
                   mon.probe(n2Core.synapseMap, [6], 'x1e1')[0]]
    ax4x1Probes = mon.probe(n2Core.synapseMap, range(8, 11), 'x1')
    ax4x2Probes = mon.probe(n2Core.synapseMap, range(8, 11), 'x2')
    uProbes = mon.probe(n2Core.cxState, range(0, 4), 'u')

    # --------------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------------
    board.run(40)

    # --------------------------------------------------------------------------
    # Plot x1 and x2 traces of each axon in figure 1006 and the 'u' of the
    # detector compartments in figure 1005.
    # --------------------------------------------------------------------------
    # Plot 'u' variable of all 4 detector compartments.
    # The diagram shows the 'u' variable for both detector compartments. When
    #  the two spikes arrive at, 'u' increases immediately by 1*2**6 for
    # compartment 0 and with a synaptic delay of 3 for compartment 1.
    if doPlot:
        fig1 = plt.figure(1005, figsize=(8, 4))
        for i in range(4):
            uProbes[i].plot()
        plt.title('u')
        plt.legend(['c[0]', 'c[1]'])

        # Plot x1 traces of axon 1 with a single SingleTraceEntry.
        # The diagram shows the trace values of the x1 trace for 2 historic
        # epochs x1e0 and x1e1. Using a tEpoch value of 2 has two effects:
        #   1. Traces are only updated every 2 time steps
        #   2. x1e1 is delayed from x1e0 by tEpoch as it reflects the state of x1
        #  in the prior epoch.
        fig2 = plt.figure(1006, figsize=(8, 8))
        plt.subplot(4, 2, 1)
        for i in range(2):
            ax1x1Probes[i].plot()
        plt.title('Axon 1, x1')
        plt.legend(['x1e0', 'x1e1'])

        # Plot x1, x2 traces of axon 2 with a single TwoTraceEntry.
        # The next two diagrams show the trace values of the x1 and x2 traced.
        # The spikeLevel for x2 is smaller and the time constant is longer to
        # distinguish it from x1.
        plt.subplot(4, 2, 3)
        ax2x1Probes[0].plot()
        plt.title('Axon 2, x1')
        plt.ylim(0, 34)
        plt.subplot(4, 2, 4)
        ax2x2Probes[0].plot()
        plt.title('Axon 2, x2')
        plt.ylim(0, 34)

        # Plot x1 traces of axon 3 with 2 SingleTraceEntries.
        # Analogous to x1 of axon 1 but for 2 SingleTraceEntries. Note: pre traces
        # accumulate spikes immediately when the spike arrives despite synapses of
        # that axon having nonzero delays. Thus pre traces are actually axonal
        # traces. The specific synaptic pre trace value is computed on demand
        # from any of the historic epochs according to the specific synaptic delay.
        plt.subplot(4, 2, 5)
        for i in range(4):
            ax3x1Probes[i].plot()
        plt.title('Axon 3, x1')
        plt.legend(['x1e0 (STE0)', 'x1e1 (STE0)',
                    'x1e0 (STE1)', 'x1e1 (STE1)'])

        # Plot x1, x2 traces of axon 4 with 3 TwoTraceEntries.
        plt.subplot(4, 2, 7)
        for i in range(3):
            ax4x1Probes[i].plot()
        plt.title('Axon 2, x1')
        plt.ylim(0, 34)
        plt.legend(['x1 (TTE0)', 'x1 (TTE1)', 'x1 (TTE2)'])

        plt.subplot(4, 2, 8)
        for i in range(3):
            ax4x2Probes[i].plot()
        plt.title('Axon 2, x2')
        plt.ylim(0, 34)
        plt.legend(['x2 (TTE0)', 'x2 (TTE1)', 'x2 (TTE2)'])

        if haveDisplay:
            plt.show()
        else:
            fileName1 = "tutorial_05_fig1005.png"
            fileName2 = "tutorial_05_fig1006.png"
            print("No display available, saving to files " +
                  fileName1 + " and " + fileName2 + ".")
            fig1.savefig(fileName1)
            fig2.savefig(fileName2)


if __name__ == "__main__":
    # --------------------------------------------------------------------------
    # Setup the network
    # --------------------------------------------------------------------------
    board = setupNetwork()

    # --------------------------------------------------------------------------
    # Run the network and plot results
    # --------------------------------------------------------------------------
    runNetwork(board, True)
    board.disconnect()
