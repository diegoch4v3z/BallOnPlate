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
# Tutorial: 10_weight_precision.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces synaptic weight precision. The weight precision
# is controlled by the number of wgtbits in synapseFmt. We configure four
# synapses each with a different synaptic weight. We illustrate the use of
# the synaptic wgtbits to adjust the precision.
#

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays

# N2Board module provides access to the neuromorphic hardware

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
    numSynapsesPerCore = [[4]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with single neuron and four synapses with varying
    # weight precision.
    # -----------------------------------------------------------------------

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

    # Configure compartment specific bias mantissa and exponent.
    # Actual numerical bias is bias*(2^biasExp) = 1*(2^6) = 64
    # vthProfile = 0 references vthProfileCfg[0]
    # cxProfile = 0 references cxProfileCfg[0]
    n2Core.cxCfg[0].configure(bias=1,
                              biasExp=6,
                              vthProfile=0,
                              cxProfile=0)

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

    # Configure synapses with different synaptic weights

    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits.
    # Here, the synaptic weight of 20 is used.
    # Each core supports 15 different types of synapse formats,
    # i.e. indices 1 to 15 where 0 is invalid. We create four different
    # synapse formats with varying number of weight bits. The four formats
    # are distinguished by synFmtId=1,2,3 or 4.
    n2Core.synapses[0].CIdx = 1
    n2Core.synapses[0].Wgt = 20
    n2Core.synapses[0].synFmtId = 1

    # This synapse connects to compartment 2 (i.e. cxCfg[2]).
    # The synaptic weight of 16 is used here.
    n2Core.synapses[1].CIdx = 2
    n2Core.synapses[1].Wgt = 16
    n2Core.synapses[1].synFmtId = 2

    # This synapse connects to compartment 3 (i.e. cxCfg[3]).
    # The synaptic weight of 10 is used here.
    n2Core.synapses[2].CIdx = 3
    n2Core.synapses[2].Wgt = 10
    n2Core.synapses[2].synFmtId = 3

    # This synapse connects to compartment 4 (i.e. cxCfg[4]).
    # The synaptic weight of 8 is used here.
    n2Core.synapses[3].CIdx = 4
    n2Core.synapses[3].Wgt = 8
    n2Core.synapses[3].synFmtId = 4

    # Configure synaptic format

    # The synaptic format details the decoding configuration needed to
    # interpret the compressed synaptic data from SYNAPSE_MEM.

    # The wgtBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of 7 maps to 8 bits, thus keeping
    # the full precision of the synaptic weight value.
    # Note, if the synapseFmt[i].fanoutType is 1 (mixed), then the sign bit
    # is included in the weight value, thereby losing one more LSB.
    # The general mapping is the following:
    #      wgtShift = 8 - wgtBits + isMixed
    #      wgt = (wgt >> wgtShift) << wgtShift

    # We configure 4 bits of precision for the first format.
    # For this example, the first synapse with synapseFmt = 1
    # has a synaptic weight of 20. We want only the 4 most significant bits,
    # i.e. 0001 0100 --> 0001 0000 (16).
    n2Core.synapseFmt[1].wgtBits = 4

    # We configure the full eight bits of precision here.
    n2Core.synapseFmt[2].wgtBits = 7

    # We configure 6 bits of precision for the third format.
    # For this example, the third synapse with synapseFmt = 3
    # has a synaptic weight of 10. For this synapseFmt we will
    # configure the mixed fanoutType, i.e. the sign bit is included
    # in the weight value. Therefore, we need to specify an extra signficiant bit
    # to get a final weight value of 8.
    # For example, without the signed bit included when we
    # set wgtBits to 5 we lose the 3 LSBs, 0000 1010 --> 0000 1000 (8).
    # However, when we configure mixed fanoutType we lose another LSB (4 total)
    # due to the inclusion of the sign bit, i.e. 0000 1010 --> 0 000 0000 (0).
    # Thus, in order to include the signed bit in the weight value (mixed fanout)
    # and get 8 as our final weight value we need to specify 6 bits, i.e.
    # 0000 1010 lose 2 LSB for precision --> 0000 1000 and 1 more LSB
    # for the sign bit --> 0 000 1000 (8).
    n2Core.synapseFmt[3].wgtBits = 6

    # We configure the full eight bits of precision here.
    n2Core.synapseFmt[4].wgtBits = 7

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
    n2Core.synapseFmt[3].numSynapses = 1
    n2Core.synapseFmt[4].numSynapses = 1

    # The idxBits field is the bit width of the compartment index value
    # ("idx") stored in synapse_mem that maps to the actual compartment
    # index CIdx stored in each synapse. CIdx is calculated by the
    # following: CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    # Valid values for idxBits are 0..7 which map
    # to 0,6,7,8,9,10,11,12. Here, idxBits of 1 maps to 6.
    n2Core.synapseFmt[1].idxBits = 1
    n2Core.synapseFmt[2].idxBits = 1
    n2Core.synapseFmt[3].idxBits = 1
    n2Core.synapseFmt[4].idxBits = 1

    # Compression indicates how the synapses are compressed. Here we
    # select dense (shared index) uncompressed.
    # 0: sparse
    # 3: dense (shared index) uncompressed
    n2Core.synapseFmt[1].compression = 0
    n2Core.synapseFmt[2].compression = 0
    n2Core.synapseFmt[3].compression = 0
    n2Core.synapseFmt[4].compression = 0

    # The fanoutType controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    n2Core.synapseFmt[1].fanoutType = 2
    n2Core.synapseFmt[2].fanoutType = 2
    n2Core.synapseFmt[3].fanoutType = 1
    n2Core.synapseFmt[4].fanoutType = 1

    return board


if __name__ == "__main__":

    # Setup the network
    board = setupNetwork()

    # Get the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    # cxState[i] contains the state of cxCfg[i].  The probe method takes
    # a list of compartment indices to obtain the data for the
    # chosen field, e.g. u or v.  The data for each compartment index
    # is returned as an element of a list.
    uProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3, 4], 'u')
    # We only monitor v for neuron 0 to show the spiking. We do not need
    # to monitor the other neurons' voltages to illustrate weight precision
    vProbes = mon.probe(n2Core.cxState, [0], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # Since there are no incoming spikes and noise is disabled by default
    # u remains constant at 0.
    fig1 = plt.figure(10)
    plt.subplot(1, 2, 1)
    uProbes[0].plot()
    plt.title('u0')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')
    # v increases due to the bias current. Upon
    # reaching the threshold of 640, the compartment resets to
    # 0 (please refer to the refractory delay tutorial for further
    # explanation of why the v plot looks like it is resetting to 128 instead
    # of 0). Since there is no refractory period, the voltage immediately
    # begins to increase again.
    plt.subplot(1, 2, 2)
    vProbes[0].plot()
    plt.title('v0')
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Synapse with synaptic weight of 20, and 4 bits of precision, i.e.
    # effectively a synaptic weight of 16.
    fig2 = plt.figure(11)
    plt.subplot(2, 1, 1)
    uProbes[1].plot()
    plt.title('wgt 20 bits 4 --> wgt 16')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 16 and full 8 bits of precision.
    # This u plot is identical to uProbes[1].
    plt.subplot(2, 1, 2)
    uProbes[2].plot()
    plt.title('wgt 16')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 10 with mixed fanout and 6 bits of precision,
    # i.e. effectively a synaptic weight of 8.
    fig3 = plt.figure(12)
    plt.subplot(2, 1, 1)
    uProbes[3].plot()
    plt.title('wgt 10 bits 6 and mixed --> wgt 8')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 8 and full 8 bits precision.
    # This u plot is idential to uProbes[3]
    plt.subplot(2, 1, 2)
    uProbes[4].plot()
    plt.title('wgt 8')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_12_fig10.png"
        fileName2 = "tutorial_12_fig11.png"
        fileName3 = "tutorial_12_fig12.png"
        print("No display available, saving to files " + fileName1 +
              ", " + fileName2 + " and " + fileName3 + ".")
        fig1.savefig(fileName1)
        fig2.savefig(fileName2)
        fig3.savefig(fileName3)
