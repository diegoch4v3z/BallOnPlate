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


def setupNetwork():

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
    numSynapsesPerCore = [[2]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with single neuron and two synapses driven by
    # a single input spike
    # -----------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartments that are used
    # are also indices 0, 1, 2 i.e. cxCfg[0],...,cxCfg[2].
    # The compartments will share the same profile.

    # The current decay (decayU) is set to 0 to highlight the box synapse behavior.
    n2Core.cxProfileCfg[0].configure(decayU=0)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] is used.
    # There are 4 comparments per compartment group within one cxMetaState.
    # The 4 compartments are distinguished by phase0, phase1, phase2
    # and phase3. The setting of the phases to 2 (PHASE_IDLE) allows the
    # compartment to be driven by an input spike.
    n2Core.cxMetaState[0].configure(phase0=2)

    # Configure threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)

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

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 2
    n2Core.synapseMap[0].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[0].synapseLen = targetSynapseLen

    # Configure the synapseMap as a discreteMapEntry, i.e. maps to synapses
    n2Core.synapseMap[0].discreteMapEntry.configure()

    # Set dsOffset explicitly to 0 to disable decay in 'u'.
    # When calculating 'u', dsOffset is added to decayU.
    # The default value of dsOffset is 1.
    n2Core.dendriteSharedCfg.configure(dsOffset=0)

    # Configure one default synapse and one box synapse

    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits. It is a normalized signed value.
    # Here, the synaptic weight of 1 is used. There are 6 bits used to
    # specify the synaptic delay offset, (i.e. 1 to 63 time step delay).
    # While the dendritic accumulator accumulates the spike instantly,
    # this will delay the accumulation in 'u' for the specified number
    # of time steps.
    # Each core supports 15 different synapse formats, i.e. indices 1 to
    # 15 where 0 is invalid.

    # This synapse connects to compartment 1 (i.e. cxCfg[1]). The synaptic
    # delay is 7, i.e. the accumulation in 'u' will be delayed 7 time steps.
    # Here we use synapse format ID 1 for the default synapse.
    n2Core.synapses[0].CIdx = 0
    n2Core.synapses[0].Wgt = 1
    n2Core.synapses[0].Dly = 7
    n2Core.synapses[0].synFmtId = 1

    # This synapse connects to compartment 2 (i.e. cxCfg[2]).
    # Here we use synapse format ID 2 for the box synapse.
    # Instead of an exponentially decaying post synaptic response
    # after receiving a spike, here we want to have a box response,
    # i.e. a constant post synaptic current for a specified amount of time.
    # To achieve this we set the decayU to zero, add the weight at t and
    # subtract the weight at t+dly. Thus we reuse the dly as the box length
    # and not as the synaptic delay.
    # Here, the box length (specified as dly) is 7.
    n2Core.synapses[1].CIdx = 1
    n2Core.synapses[1].Wgt = 1
    n2Core.synapses[1].Dly = 7
    n2Core.synapses[1].synFmtId = 2

    # Configure synaptic format

    # The synaptic format details the decoding configuration needed to
    # interpret the compressed synaptic data from SYNAPSE_MEM.

    # The wgtBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of 7 maps to 8 bits, thus keeping
    # the full precision of the synaptic weight value.
    n2Core.synapseFmt[1].wgtBits = 7
    n2Core.synapseFmt[2].wgtBits = 7

    # The dlyBits field specifies the number of significant bits used for
    # the synaptic delay value. Valid values are 0..6.
    n2Core.synapseFmt[1].dlyBits = 6
    n2Core.synapseFmt[2].dlyBits = 6

    # Setting numSynapses specifies a certain form of encoding.
    # The value of 1 means there is 1 synapse and the user has specified
    # the mapping. The value of 63 is a special value and does not mean 63
    # synapses, but instead is a special value to have
    # the number of synapses encoded in the prefix field of synapse_mem.
    # In other words, using 63 indicates to the compiler to figure out how
    # to map synapses into synapse entries, where as setting a different
    # value for numSynapses indicates that the user specifies the mapping.
    n2Core.synapseFmt[1].numSynapses = 1
    n2Core.synapseFmt[2].numSynapses = 1

    # The idxBits field is the bit width of the compartment index value
    # ("idx") stored in synapse_mem that maps to the actual compartment
    # index CIdx stored in each synapse. CIdx is calculated by the
    # following: CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    # Valid values for idxBits are 0..7 which map
    # to 0,6,7,8,9,10,11,12. Here, idxBits of 1 maps to 6.
    n2Core.synapseFmt[1].idxBits = 1
    n2Core.synapseFmt[2].idxBits = 1

    # Compression indicates how the synapses are compressed. Here we
    # select sparse.
    # 0: sparse
    # 3: dense (shared index) uncompressed
    n2Core.synapseFmt[1].compression = 0
    n2Core.synapseFmt[2].compression = 0

    # The fanoutType controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    n2Core.synapseFmt[1].fanoutType = 2
    n2Core.synapseFmt[2].fanoutType = 2

    # Set synapseFmt[i].synType to enable box synapse format
    n2Core.synapseFmt[2].synType = 1

    return board


if __name__ == "__main__":

    # Setup the network
    board = setupNetwork()

    # Get the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # Generate an input spike at timestep 3 to chip 0, core 4 and axon 0
    bs = BasicSpikeGenerator(board)
    timestep = 3
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
    uProbes = mon.probe(n2Core.cxState, [0, 1], 'u')
    #vProbes = mon.probe(n2Core.cxState, [0, 1], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # The neuron is driven by a single spike inserted at time step 3.
    # Each synapse illustrates the manifestation of this spike depending
    # on the synapse type.

    # The spike arrives at time step 3, but with a synaptic delay of 7
    # u does not accumulate until after the synaptic delay. u remains
    # constant after accumulating due to the decayU = 0.
    fig = plt.figure(10)
    plt.subplot(2, 1, 1)
    uProbes[0].plot()
    plt.title('Synapse with synaptic delay')
    plt.ylabel('Membrane current')

    # The spike arrives at time step 3 when the weight is immediately
    # accumulated in u. After the specified time (7 in this tutorial)
    # the weight is subtracted bringing u back to 0.
    plt.subplot(2, 1, 2)
    uProbes[1].plot()
    plt.title('Box synapse')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_11_fig10.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)

