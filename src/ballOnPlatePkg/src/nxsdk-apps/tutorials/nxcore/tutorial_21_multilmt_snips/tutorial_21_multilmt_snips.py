"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

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
# Tutorial: tutorial_21_multilmt_snips.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces the snip creation api along with channel api and
# demonstrates how can a user assign a snip to particular chip and lakemont
# (x86 processor).
#
# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------
from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
# N2Board module provides access to the neuromorphic hardware
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays

# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function sets up the network and returns the board
# -----------------------------------------------------------------------------


def setupNetwork():
    # -------------------------------------------------------------------------
    # Initialize board
    # -------------------------------------------------------------------------

    # Board ID
    boardId = 1
    # Number of chips
    numChips = 2
    # Number of cores per chip
    numCoresPerChip = [1, 1]
    # Number of synapses per core
    numSynapsesPerCore = [[2], [0]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure cx 0 driven by bias only
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartment that is driven by
    # bias current is also index 0, i.e. cxCfg[0].
    n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12),
                                     decayU=int(1/10*2**12))

    # Setting compartment to be driven by a bias current alone.
    n2Core.cxMetaState[0].configure(phase0=2)

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)

    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group).
    n2Core.numUpdates.configure(numUpdates=1)

    # Configure bias mantissa and exponent.
    n2Core.cxCfg[0].configure(
        bias=1,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # -------------------------------------------------------------------------
    # Connect compartment 0 to compartment 1 and compartment 0
    # -------------------------------------------------------------------------

    # Configure output axon

    axonCfgPtr = 0
    axonCfgLen = 1
    n2Core.axonMap[0].configure(
        ptr=axonCfgPtr,
        len=axonCfgLen)

    # Setup the connection to the synapseMap
    targetCoreId = 4
    targetAxonId = 0
    n2Core.axonCfg[axonCfgPtr].discrete.configure(
        coreId=targetCoreId,
        axonId=targetAxonId)

    # Configure input axon
    targetSynapsePtr = 0
    targetSynapseLen = 2
    n2Core.synapseMap[targetAxonId].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[targetAxonId].synapseLen = targetSynapseLen
    # Set the synapseMap as a discreteMapEntry, i.e. simply maps to synapses.
    n2Core.synapseMap[targetAxonId].discreteMapEntry.configure()

    # Configure synapses
    n2Core.synapses[0].CIdx = 1
    n2Core.synapses[0].Wgt = 4
    n2Core.synapses[0].synFmtId = 1

    # This synapse connects to compartment 0 (i.e. cxCfg[0]).
    n2Core.synapses[1].CIdx = 0
    n2Core.synapses[1].Wgt = 4
    n2Core.synapses[1].synFmtId = 1

    # Configure synFmt
    n2Core.synapseFmt[1].wgtExp = 0
    n2Core.synapseFmt[1].wgtBits = 7
    n2Core.synapseFmt[1].numSynapses = 63
    n2Core.synapseFmt[1].idxBits = 1
    n2Core.synapseFmt[1].compression = 0
    n2Core.synapseFmt[1].fanoutType = 1
    return board


def setupSnipsAndRunNetwork(board):
    # Defining snips which will be ran during different phases
    # createProcess takes following arguments :
    # param nxGraph: Board with which snip is associated
    # param name: Name of the snip
    # param cFilePath: Path to the c file where C code for snip exists
    # param includeDir: Path to the include file associated with snip C code
    # param funcName: Function name in the C file
    # param guardName: Function name in the C file to be used as guard
    # param phase: phase in which the snips is going to be inserted
    # param platform: platform on which snip will be executed
    # param chipId: Chip id on which the snip will run
    # param lmtId: id of the lakemont on which snip will run if platform is lakemont
    # return: process
    # Default value of chipId and lmtId is 0

    # Defining a process which will run during init phase
    # This snip will run on chip 0 lmt 1
    initProcess = board.createProcess(
        name="initprocess",
        cFilePath=os.path.dirname(os.path.realpath(__file__)) + "/initsnip.c",
        includeDir=os.path.dirname(os.path.realpath(__file__)),
        funcName="initsnip",
        guardName=None,
        phase="init",
        lmtId=1
    )

    # Defining a process which will run during spiking phase
    # This snip will run on chip 1 lmt 0
    spikingProcess = board.createProcess(
        name="spikingProcess",
        cFilePath=os.path.dirname(os.path.realpath(__file__)) + "/spiking.c",
        includeDir=os.path.dirname(os.path.realpath(__file__)),
        funcName="run_spiking",
        guardName="do_spiking",
        phase="spiking",
        chipId=1,
        lmtId=0
    )

    # Defining a process which will during run mgmt phase
    # This snip will run on chip 0 lmt 0
    runMgmtProcess = board.createProcess(
        name="runMgmt",
        cFilePath=os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c",
        includeDir=os.path.dirname(os.path.realpath(__file__)),
        funcName="run_mgmt",
        guardName="do_run_mgmt",
        phase="mgmt",
    )

    # Creating channels which serves as a channel of communication between lmt/superhost
    # Create Channel API takes following params :
    # param name: Channel Name
    # param elementType: Only support "int" for now
    # param numElements: Maximum capacity of the channel
    # param behavior: Not implemented
    # return: channel

    # Creating a channel named spikeChannel for sending spike data
    spikeChannel = board.createChannel(b'nxspiking', "int", 30)
    # Connecting spikechannel from Superhost to spikingProcess making it send channel
    spikeChannel.connect(None, spikingProcess)

    # Create a channel named initChannel for getting the value of tEpoch
    initChannel = board.createChannel(b'nxinit', "int", 30)
    # Connecting initChannel from initProcess to SuperHost making it receive channel
    initChannel.connect(initProcess, None)

    # Plot to see the spikes
    # Configure probes
    mon = board.monitor
    n2Core = board.n2Chips[0].n2Cores[0]
    uProbe1 = mon.probe(n2Core.cxState, [0], 'u')[0]
    vProbe1 = mon.probe(n2Core.cxState, [0], 'v')[0]
    uProbe2 = mon.probe(n2Core.cxState, [1], 'u')[0]
    vProbe2 = mon.probe(n2Core.cxState, [1], 'v')[0]

    # Set the learning epoch as 2
    board.options = ['epoch=2']
    board.run(10, aSync=True)

    # From the initsnip.c, we are returning the numupdates value via "nxinit" channel
    numUpdates = initChannel.read(1)
    print("Value of NumUpdates is : {}".format(numUpdates[0]))
    for i in range(10):
        # Sending spikes to snip process via nxspiking channel
        spikeChannel.write(3, [0, 4, 0])  # ChipId,CoreId,AxonId
    board.finishRun()
    board.disconnect()

    # ------------------------------------------------------------------------------------
    # Plots
    # ------------------------------------------------------------------------------------
    # 2 snips running on 2 different lmts to change the state of the cx 0 and cx1.
    # runMgmt snip resets the state of the cx0 and cx1 every 3rd cycle
    # spiking snip inserts spike on 0th axon of the core on which cx0 and cx0 are located
    # In the plot of u,v of cx 0 and 1, we can see spikes being inserted at every cycle and
    # value of u,v being reset every third cycle.

    fig = plt.figure(1)
    plt.subplot(2, 2, 1)
    plt.plot(uProbe1.timeSeries.data)
    plt.title('u1')

    plt.subplot(2, 2, 2)
    plt.plot(vProbe1.timeSeries.data)
    plt.title('v1')

    plt.subplot(2, 2, 3)
    plt.plot(uProbe2.timeSeries.data)
    plt.title('u2')

    plt.subplot(2, 2, 4)
    plt.plot(vProbe2.timeSeries.data)
    plt.title('v2')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_21_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)


if __name__ == "__main__":
    board = setupNetwork()
    setupSnipsAndRunNetwork(board)
