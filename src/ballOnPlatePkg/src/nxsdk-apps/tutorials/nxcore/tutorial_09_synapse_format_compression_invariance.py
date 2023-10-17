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
# Tutorial: tutorial_09_synapse_format_compression_invariance.py
# -----------------------------------------------------------------------------
# This tutorial configures a single compartment and drives the voltage using
# only a bias current.  This compartment is then connected to 250 other
# compartments with 250 synapses.  The first 125 synapses use sparse
# compression and the second 125 synapses use dense compression.  From the
# resulting plots, it can be seen that the use of dense or sparse compression
# does not affect the behavior.

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

import math
from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays
# N2Board module provides access to the neuromorphic hardware
# import math

# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function sets up the network and returns the board
# Note: For this illustration, numTestSynapses must be even
# -----------------------------------------------------------------------------


def setupNetwork(numTestSynapses):
    # -------------------------------------------------------------------------
    # Initialize board
    # -------------------------------------------------------------------------
    assert (numTestSynapses % 2 == 0)

    # Board ID
    boardId = 1
    # Number of chips
    numChips = 1
    # Number of cores per chip
    numCoresPerChip = [1]
    # Number of synapses per core
    numSynapsesPerCore = [[numTestSynapses]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure core with 1 compartment driven by bias only
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartment that is driven by
    # bias current is also index 0, i.e. cxCfg[0].

    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    # Current decay is set to 1/10 (2^12 factor for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12),
                                     decayU=int(1/10*2**12))
    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0 is used.
    # The setting of phase0 = PHASE_IDLE = 2 allows the compartment to be
    # driven by a bias current alone.  See user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to compartments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=math.ceil((numTestSynapses+1)/4))
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 1*(2^6) = 64.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=1,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # -------------------------------------------------------------------------
    # Connect compartment 0 to compartments 1 through numTestSynapses+1
    # -------------------------------------------------------------------------

    # Configure output axon
    targetCoreId = 4
    targetAxonId = 0
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(0, targetChipId, targetCoreId, targetAxonId)

    # Configure input axon

    # Setup the connection to the synapses.  Similar to the axonMap above, the
    # targetSynapsePtr indexes into synapses and targetSynapseLen denotes how
    # many consecutive indices in synapses (starting from targetSynapsePtr) are
    # connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = numTestSynapses
    n2Core.synapseMap[targetAxonId].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[targetAxonId].synapseLen = targetSynapseLen
    # Set the synapseMap as a discreteMapEntry, i.e. simply maps to synapses.
    n2Core.synapseMap[targetAxonId].discreteMapEntry.configure()

    # Configure synapses

    # Connected synapse i to compartment i+1
    # First half of the synapses use synapse format 1 and the second half uses
    # synapse format 2.
    for loop in range(0, numTestSynapses):
        n2Core.synapses[loop].CIdx = loop+1
        n2Core.synapses[loop].Wgt = 4
        if loop < numTestSynapses/2:
            n2Core.synapses[loop].synFmtId = 1
        else:
            n2Core.synapses[loop].synFmtId = 2

    # Configure synFmt

    # The wgtExp field allows for the additional up or down bit shifting of the
    # Wgt parameter for normalization purposes.  Here, no additional shifts are
    # made.
    n2Core.synapseFmt[1].wgtExp = 0
    # The wgtBits field specifies the number of significant bits used for the
    # Wgt value.  A setting of 7 maps to 8 bits, thus keeping the full
    # precision of the Wgt value.
    n2Core.synapseFmt[1].wgtBits = 7
    # Setting numSynapses specifies a certain form of encoding.  The value of
    # 63 does not mean 63 synapses, but instead is a special value to have the
    # number of synapses encoded in the prefix field of synapse_mem.  To
    # describe this another way, using 63 asks the compiler figure out how to
    # map synapses into synapse entries, but setting another value for
    # numSynapses mean that the user specifies the mapping.
    n2Core.synapseFmt[1].numSynapses = 63
    # idxBits set to 3 maps to 8 bits.  This is the bit width of the
    # compartment index value ("idx") stored in synapse_mem that maps to the
    # actual compartment index CIdx stored in each synapse.  The mapping is
    # CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    n2Core.synapseFmt[1].idxBits = 3
    # Selects how synapses are compressed. Compression setting of 0 means
    # sparse compression.
    n2Core.synapseFmt[1].compression = 0
    # fanoutType of 1 indicates mixed fanout, i.e. no shared sign bit (included
    # in each weight value)
    n2Core.synapseFmt[1].fanoutType = 1

    n2Core.synapseFmt[2].wgtExp = 0
    n2Core.synapseFmt[2].wgtBits = 7
    n2Core.synapseFmt[2].numSynapses = 63
    n2Core.synapseFmt[2].idxBits = 3
    # Compression setting of 3 means dense compression.
    n2Core.synapseFmt[2].compression = 3
    n2Core.synapseFmt[2].fanoutType = 1

    # Return the configured board
    return board


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    # numTestSynapses is total number of synapses.  Half will be used with
    # sparse compression and the other half will use dense compression.
    # For this tutorial, make numTestSynapses even.
    numTestSynapses = 250
    # Setup the network
    board = setupNetwork(numTestSynapses)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    # cxState[i] contains the state of cxCfg[i].  The probe method takes a list
    # of compartment indices to obtain the data for the chosen field, e.g. u or
    # v.  The data for each compartment index is returned as an element of a
    # list.
    mon = board.monitor
    uProbes = mon.probe(n2Core.cxState, [0, 1, int(numTestSynapses/2+1)], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, int(numTestSynapses/2+1)], 'v')

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(30)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    # The plot for the bias driven compartment 0 is shown on the first row.
    fig = plt.figure(1002)
    plt.subplot(3, 2, 1)
    uProbes[0].plot()
    plt.title('u0 bias driven')
    plt.subplot(3, 2, 2)
    vProbes[0].plot()
    plt.title('v0 bias driven')

    # The plot for the first compartment using sparse compression is shown on
    # the middle row.
    plt.subplot(3, 2, 3)
    uProbes[1].plot()
    plt.title('u1 sparse compression')
    plt.subplot(3, 2, 4)
    vProbes[1].plot()
    plt.title('v1 sparse compression')

    # The plot for the first compartment using dense compression is shown on
    # the bottom row.
    plt.subplot(3, 2, 5)
    uProbes[2].plot()
    plt.title('u'+str(int(numTestSynapses/2+1))+' dense compression')
    plt.subplot(3, 2, 6)
    vProbes[2].plot()
    plt.title('v'+str(int(numTestSynapses/2+1))+' dense compression')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_09_fig1002.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
