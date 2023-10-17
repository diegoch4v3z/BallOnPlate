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
# This tutorial introduces synaptic delays and will demonstrate how they work and
# how to configure them. We configure five compartments driven by an input
# spike generator. The first compartment is connected to four other compartments
# through four synapses - each with a different synaptic delay.
# We inject three spikes to the first compartment (here at timesteps 5, 15, and 30).
# The first synapse has no synpatic delay, while the second,
# third, and fourth synapses have 12, 28, and 62 timestep delays respectively
# (these numbers are randomly chosen for illustration purposes).
# This tutorial also details the configuration of the balance between the
# synaptic delay precision and the number of dendritic compartments.
#

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

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

# Input generation module used to insert a basic spike

# Define a function to setup the network


def setupNetwork(synDlys):

    # -----------------------------------------------------------------------
    # Initialize board
    # -----------------------------------------------------------------------

    # Board ID
    boardId = 1

    # Number of chips
    numChips = 1

    # Number of cores per chip
    numCoresPerChip = [1]

    # Number of synapses per core
    numSynapsesPerCore = [[5]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with 5 compartments driven by input spikes
    # with varying synaptic delays
    # -----------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartments that are used
    # are also indices 0, 1, 2, 3, 4 i.e. cxCfg[0],...,cxCfg[4].
    # The compartments will share the same profile.

    # Voltage decay (decayV) is set to 1/16 and the current decay (decayU)
    # is set to 1/10 (2^12 factor is for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12),
                                     decayU=int(1/10*2**12))

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0
    # is used. The setting of phase0 = PHASE_IDLE = 2 allows the
    # compartment to be driven by an input spike.
    # See the user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)

    # Configure threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=2)

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
    targetCoreId = 4
    targetAxonId = 0
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(0, targetChipId, targetCoreId, targetAxonId)

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 4
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
    n2Core.synapses[0].CIdx = 1
    n2Core.synapses[0].Wgt = 1
    n2Core.synapses[0].Dly = synDlys[0]
    n2Core.synapses[0].synFmtId = 1

    # This synapse connects to compartment 2 (i.e. cxCfg[2]).
    # Here, the synaptic delay is 12, i.e. the accumulation in 'u' will be
    # delayed 12 timesteps.
    n2Core.synapses[1].CIdx = 2
    n2Core.synapses[1].Wgt = 1
    n2Core.synapses[1].Dly = synDlys[1]
    n2Core.synapses[1].synFmtId = 1

    # This synapse connects to compartment 3 (i.e. cxCfg[3]).
    # Here, the synaptic delay is 28, i.e. the accumulation in 'u' will be
    # delayed 28 timesteps.
    n2Core.synapses[2].CIdx = 3
    n2Core.synapses[2].Wgt = 1
    n2Core.synapses[2].Dly = synDlys[2]
    n2Core.synapses[2].synFmtId = 1

    # This synapse connects to compartment 4 (i.e. cxCfg[4]).
    # Here, the synaptic delay is 62, i.e. the accumulation in 'u' will be
    # delayed 62 timesteps.
    n2Core.synapses[3].CIdx = 4
    n2Core.synapses[3].Wgt = 1
    n2Core.synapses[3].Dly = synDlys[3]
    n2Core.synapses[3].synFmtId = 1

    # Configure synaptic format

    # The synaptic format details the decoding configuration needed to
    # interpret the compressed synaptic data from SYNAPSE_MEM.
    # The wgtExp field allows for additional up or down bit shifting of
    # the synaptic weight for normalization purposes. Here, no additional
    # shifting is done.
    n2Core.synapseFmt[1].wgtExp = 0

    # The wgtBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of 7 maps to 8 bits, thus keeping
    # the full precision of the synaptic weight value.
    n2Core.synapseFmt[1].wgtBits = 7

    # The dlyBits field specifies the number of significant bits used for
    # the synaptic delay value. Valid values are 0..6.
    n2Core.synapseFmt[1].dlyBits = 6

    # Setting numSynapses specifies a certain form of encoding.
    # The value of 4 means there are 4 synapses and the user has specified
    # the mapping. The value of 63 is a special value and does not mean 63
    # synapses, but instead is a special value to have
    # the number of synapses encoded in the prefix field of synapse_mem.
    # In other words, using 63 indicates to the compiler to figure out how
    # to map synapses into synapse entries, where as setting a different
    # value for numSynapses indicates that the user specifies the mapping.
    n2Core.synapseFmt[1].numSynapses = 4

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
    n2Core.synapseFmt[1].compression = 3

    # The fanoutType controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    n2Core.synapseFmt[1].fanoutType = 2

    return board


if __name__ == "__main__":

    # List of varying synaptic delays
    synDlys = (0, 12, 28, 62)

    # Setup the network
    board = setupNetwork(synDlys)

    # Get the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # Generate an input spike at timestep 5 to chip 0, core 4 and axon 0
    bs = BasicSpikeGenerator(board)
    timestep = 5
    chipId = 0
    coreId = 4
    axonId = 0
    bs.addSpike(timestep, chipId, coreId, axonId)

    # Generate an input spike at timestep 15 to chip 0, core 4 and axon 0
    timestep = 15
    chipId = 0
    coreId = 4
    axonId = 0
    bs.addSpike(timestep, chipId, coreId, axonId)

    # Generate an input spike at timestep 30 to chip 0, core 4 and axon 0
    timestep = 30
    chipId = 0
    coreId = 4
    axonId = 0
    bs.addSpike(timestep, chipId, coreId, axonId)

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    # cxState[i] contains the state of cxCfg[i].  The probe method takes
    # a list of compartment indices to obtain the data for the
    # chosen field, e.g. u or v.  The data for each compartment index
    # is returned as an element of a list.
    uProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3, 4], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3, 4], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # The current variable u jumps as the spikes come in for the first
    # synapse. As the delay period elapses for the second, third and fourth
    # synapse the current variable u accumulates and increases.
    # Similarly, the voltage variable v will grow as the synaptic delay
    # periods elapse and will then start to decay.
    fig = plt.figure(1004, figsize=(30, 20))
    k = 1
    numReceiverNeurons = 4
    # For each compartment plot the u and v variables
    for j in range(0, numReceiverNeurons+1):
        plt.subplot(numReceiverNeurons+1, 2, k)
        uProbes[j].plot()
        plt.title('u'+str(j))
        k += 1
        plt.subplot(numReceiverNeurons+1, 2, k)
        vProbes[j].plot()
        plt.title('v'+str(j))
        k += 1

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_04_fig1004.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
