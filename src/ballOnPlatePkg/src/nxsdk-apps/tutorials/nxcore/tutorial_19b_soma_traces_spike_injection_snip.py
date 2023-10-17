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
# Tutorial: tutorial_19b_soma_traces_spike_injection_snip.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces soma traces and will demonstrate how they work and
# how to configure them. We configure 1 compartment on 6 different cores
# driven by input spikes. 6 different cores are needed because\
# this tutorial demonstrates the usage of time constants and epoch time steps
# and these values are shared for each compartment per core.
# We inject four spikes to the compartment (here at timesteps
#  5, 25, 45 and 65) using a spike injection SNIP and channel.
# The thresholds are setup that each input spike elicits a spike of the
# compartment. The trace of the current, spikes and soma for different
# time constants is probed.
#

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from nxsdk.arch.n2a.n2board import N2Board
import nxsdk
import matplotlib.pyplot as plt
import os
import matplotlib as mpl

haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays

# N2Board module provides access to the neuromorphic hardware

# Input generation module used to insert a basic spike


# Define a function to setup the network
def setupNetwork():
    # -----------------------------------------------------------------------
    # Initialize board
    # -----------------------------------------------------------------------

    # Board ID
    boardId = 1

    # Number of chips
    numChips = 1

    # Number of cores per chip
    numCoresPerChip = [6]

    # Number of synapses per core
    numSynapsesPerCore = [[1, 1, 1, 1, 1, 1]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # -----------------------------------------------------------------------
    # Configure 6 cores with 1 compartments driven by input spikes and
    # activated homeostasis
    # 3 cores have a time constant tau of 8 and 3 Cores have a tau of 16
    # with 1, 8 and 16 respective time steps before the soma gets updated.
    # A t_epoch of 1 means, that at each time step the soma value gets
    # evaluated, which is not very efficiant. At t_epoch=8 the soma trace
    # is only evaluated every 8th time step.
    # -----------------------------------------------------------------------

    # Obtain the relevant cores (six in total)
    n2CoreList = board.n2Chips[0].n2CoresAsList

    tauList = [8, 8, 8, 16, 16, 16]
    t_epochList = [1, 8, 16, 1, 8, 16]

    j = 0
    for n2Core in n2CoreList:
        configureCore(n2Core, tauList[j], t_epochList[j])
        j = j + 1

    return board

# Define spike channel
def setupSpkChannel(board):

    # Create a process and channel for injecting spikes
    cPath = os.path.join(nxsdk.__path__[0], "snips", "spiking.c")
    includeDir = os.path.join(nxsdk.__path__[0], "snips")

    funcName = "runSpiking"
    guardName = "doSpiking"
    phase = "spiking"
    spiking = board.createProcess(
        "spiking", cPath, includeDir, funcName, guardName, phase)
    spkChannel = board.createChannel(b'nxspk', "int", 200)
    spkChannel.connect(None, spiking)

    return spkChannel


# Define a function to configure a given core with different
# decay time constants and epoch time constants
# The time constants are shared by core, therefore we need to
# setup a different core for each of them.
def configureCore(n2Core, tau, t_epoch):
    """

    :param n2Core: the core to configure
    :param tau: the time constant to use for the soma trace
    :param t_epoch: amount of steps after the soma trace gets updated
    """

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartments that are used
    # are also indices 0, 1, 2, 3, 4 i.e. cxCfg[0],...,cxCfg[4].
    # The compartments will share the same profile.

    # Voltage decay (decayV) is set to 1/16 and the current decay (decayU)
    # is set to 1/10 (2^12 factor is for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1 / 16 * 2 ** 12),
                                     decayU=int(1 / 10 * 2 ** 12))

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0
    # is used. The setting of phase0 = PHASE_IDLE = 2 allows the
    # compartment to be driven by an input spike.
    # See the user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)

    # Initialize vthProfileCfg
    # Homeostasis needs to be enabled (=1) to get somaTraces
    # Since beta=0 and aMin and aMax are set to their minimum and
    # maximum value, the threshold should remain constant.
    # Thus, in this example the homeostasis does not change anything.
    n2Core.vthProfileCfg[0].dynamicCfg.configure(enableHomeostasis=1,
                                                 beta=0,
                                                 aMin=0,
                                                 aMax=127)

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 4*2^6 = 256
    n2Core.somaState[0].configure(vth=4)

    # Initialize dendriteTimeState
    n2Core.dendriteTimeState[0].tepoch = t_epoch

    # Initialize somaTraceCfg
    # Configure pre trace behavior (trace decay time constant and the level
    # by which a trace gets incremented upon each spike). Since the register
    # encoding of this information is quite complex, we use a helper class
    # (TraceCfgGen) that generates the register configuration based on the time
    # constant (tau) and increment (spikeLevel) and writes it to the appropriate
    # registers.
    tcg = TraceCfgGen()
    tc = tcg.genTraceCfg(tau=tau,
                         spikeLevelInt=40,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.somaTraceCfg[0])

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # Configure the dendrite accumulator delay range.
    # This controls the balance between the delay resolution and the
    # number of dendritic compartments.
    # There are 2**13 dendritic accumulators. delayBits controls the
    # maximum number of delay buckets per compartment and therefore the
    # maximum number of addressable compartments. The number of delay
    # buckets is 2**delayBits-1 and the number of addressable compartments
    # is 2**(13-delayBits), thus the number of accumulators is 2**13.
    # As you allocate more bits for the delay
    # precision, there are less bits to address the dendritic compartment.
    # For example, here we configure the delayBits to be 6, giving us up
    # to 63 delay buckets per compartment and 2**7(128) compartments.
    # A delayBits value of 0 is interpreted as 1 and 7 is interpreted as
    # 6 (maximum supported). When delayBits is not set the default is 4,
    # thus there are 512 compartments with 16-1 delay buckets.
    # Specify less delay bits in order to increase the number of
    # dendritic compartments and similarly specify more delay bits
    # in order to increase the synaptic delay resolution.
    n2Core.dendriteAccumCfg.delayBits = 6

    # Configure output axon

    # There is a 1:1 mapping between axonMap and cxCfg.
    # Configure the axonMap associated with cxCfg[0], i.e. axonMap[0].
    # axonCfgPtr is the index into axonCfg[] and axonCfgLen denotes
    # how many consecutive indices in axonCfg (starting from axonCfgPtr)
    # are connected to compartment cxCfg[0]. axonCfg corresponds to output
    # axons connecting to input axons on destination cores.
    # Specifically, axonCfg[axonCfgPtr], axonCfg[axonCfgPtr+1],
    # axonCfg[axonCfgPtr+2],..., axonCfg[axonCfgLen-1] are connected to
    # compartment cxCfg[0].
    axonCfgPtr = 0
    axonCfgLen = 1
    n2Core.axonMap[0].configure(ptr=axonCfgPtr,
                                len=axonCfgLen)

    # Setup the connection to the synapseMap (e.g. "input axon"). The
    # targetCoreId is the ID of the core where the input axon is located.
    # Core ID of 4 is the first core (see user guide for core ID
    # specification).
    # The targetAxonId is the index into synapseMap.
    targetCoreId = 4
    targetAxonId = 0
    n2Core.axonCfg[0].discrete.configure(coreId=targetCoreId,
                                         axonId=targetAxonId)

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 1
    n2Core.synapseMap[0].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[0].synapseLen = targetSynapseLen

    # Configure the synapseMap as a discreteMapEntry, i.e. maps to synapses
    n2Core.synapseMap[0].discreteMapEntry.configure()

    # Configure synapses with different synaptic delays

    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits. It is a normalized signed value.
    # Here, the synaptic weight of 1 is used. There are 6 bits used to
    # specify the synaptic delay offset, (i.e. 1 to 63 time step delay).
    # While the dendritic accumulator accumulates the spike instantly,
    # this will delay the accumulation in 'u' for the specified number
    # of time steps. For this synapse there is no delay.
    # Each core supports 15 different types of synapse formats, i.e. indices 1 to
    # 15 where 0 is invalid. In this example synapse format ID 1 is used.
    n2Core.synapses[0].CIdx = 0
    n2Core.synapses[0].Wgt = 1
    n2Core.synapses[0].synFmtId = 1

    # Configure synaptic format

    # The wgtBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of 7 maps to 8 bits, thus keeping
    # the full precision of the synaptic weight value.
    n2Core.synapseFmt[1].wgtBits = 7

    # Setting numSynapses specifies a certain form of encoding.
    # The value of 4 means there are 4 synapses and the user has specified
    # the mapping. The value of 63 is a special value and does not mean 63
    # synapses, but instead is a special value to have
    # the number of synapses encoded in the prefix field of synapse_mem.
    # In other words, using 63 indicates to the compiler to figure out how
    # to map synapses into synapse entries, where as setting a different
    # value for numSynapses indicates that the user specifies the mapping.
    n2Core.synapseFmt[1].numSynapses = 1

    # The idxBits field is the bit width of the compartment index value
    # ("idx") stored in synapse_mem that maps to the actual compartment
    # index CIdx stored in each synapse. CIdx is calculated by the
    # following: CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    # Valid values for idxBits are 0..7 which map
    # to 0,6,7,8,9,10,11,12. Here, idxBits of 1 maps to 6.
    n2Core.synapseFmt[1].idxBits = 1

    # Compression indicates how the synapses are compressed. Here we
    # select dense (shared index) uncompressed.
    # 0: sparse
    # 1: runlength compressed (shared index with offset)
    # 2: unused
    # 3: dense (shared index) uncompressed
    # 4: zero-mask compression (shared index with mask)
    n2Core.synapseFmt[1].compression = 0 # sparse

    # The fanoutType controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    n2Core.synapseFmt[1].fanoutType = 2


# Configures the spike generator in a way that every core is provided
# with the same input spikes at spiketimes
def configureSpikeGenerator(board, coreList, spkChannel, spiketimes):
    """

    :param board: the board to setup BasicSpikeGenerator
    :param n2CoreList: list of cores
    :param spiketimes: a list of times for the spikes
    """

    # Write the spike injection data to the channel
    # Inject the spikes to each core for each time in the spiketimes list
    # Data must be sorted by time
    data = []
    for time in spiketimes:
        for n2Core in coreList:
            chipId = 0
            coreId = n2Core.id
            axonId = 0
            data.append(time)
            data.append(chipId)
            data.append(coreId)
            data.append(axonId)
    data.append(-1)

    board.start()

    spkChannel.write(len(data), data)


# Configure the probes in order to monitor the traces of u, spike an a
# Sets up a list of probes for each core
def configureProbes(monitor, n2CoreList):
    """

    :param monitor:
    :param n2Core:
    :return: uProbesList, spikeProbesList, somaProbes
    """

    uProbesList = []
    spikeProbesList = []
    somaProbesList = []

    for n2Core in n2CoreList:

        # cxState[i] contains the state of cxCfg[i].  The probe method takes
        # a list of compartment indices to obtain the data for the
        # chosen field, e.g. u or v.
        # somaState[i] can be used to obtain data for the soma trace (a).
        # The data for each compartment index is returned as an element of a
        # list.
        uProbesList.append(monitor.probe(n2Core.cxState, [0], 'u')[0])
        spikeProbesList.append(monitor.probe(n2Core.cxState, [0], 'spike')[0])
        somaProbesList.append(monitor.probe(n2Core.somaState, [0], 'a')[0])

    return uProbesList, spikeProbesList, somaProbesList


if __name__ == "__main__":

    # Setup the network
    board, n2CoreList, spkChannel = setupNetwork()

    board.sync = True

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    uProbesList, spikeProbesList, somaProbesList = configureProbes(
        mon, n2CoreList)

    configureSpikeGenerator(board, n2CoreList, spkChannel, [5, 25, 45, 65])

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # The current variable u jumps as the spikes come in for the synapse.
    # Similarly, the voltage variable v will grow and reach the threshold
    # to elicit a spike, for each input spike. Also the soma voltage
    # grows with each spike and decays afterwards, depending on the
    # chosen decay time constant tau. A higher time constant tau leads to
    # a slower decay of the soma trace. It is also obserable that a higher
    # value of tepoch leads to fewer evaluations of the soma trace.
    fig = plt.figure(1019)
    k = 1
    numCores = len(n2CoreList)
    # For each compartment plot the u, v, spike and a variables
    j = 0
    tauList = [8, 8, 8, 16, 16, 16]
    t_epochList = [1, 8, 16, 1, 8, 16]
    for j in range(0, numCores):
        plt.subplot(numCores, 3, k)
        uProbesList[j].plot()
        plt.title('u' + str(j))
        k += 1
        plt.subplot(numCores, 3, k)
        spikeProbesList[j].plot()
        plt.title('spikes' + str(j))
        k += 1
        plt.subplot(numCores, 3, k)
        somaProbesList[j].plot()
        plt.title('a' + str(j))
        plt.legend(["tepoch = " + str(t_epochList[j])])
        k += 1

    fig.text(0.94, 0.5, '<-- tau=16        tau=8  -->',
             ha='center', va='center', rotation='vertical')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_19_fig1019.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
