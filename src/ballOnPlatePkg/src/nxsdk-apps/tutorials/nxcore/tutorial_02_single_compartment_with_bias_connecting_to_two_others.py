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
# Tutorial: tutorial_02_single_compartment_with_bias_connecting_to_two_others.py
# -----------------------------------------------------------------------------
# This tutorial configures a single compartment and drives the voltage using
# only a bias current.  This compartment is then connected to two additional
# compartments, one through an excitatory synapse and one through an
# inhibitory synapse.
#
# Bias current driven compartment, i.e. compartment 0:
# The behavior of the bias current driven compartment is described in tutorial
# 01.
#
# Compartment connected via excitatory synapse, i.e. compartment 1:
# When compartment 0 spikes at around t = 15, it causes the current for
# compartment 1 to jump.  This results in the voltage for compartment 1
# to increase and then spike.  The voltage then immediately begins to increase
# again since the current is still positive, though slowly decreasing.  The
# voltage reaches threshold and spikes again and the voltage again starts to
# increase.  Note that the rate of voltage increase after the second spike
# is slower than that after the first spike since the current value is
# decreasing.
#
# Compartment connected via inhibitory synapse, i.e. compartment 2:
# When compartment 0 spikes at around t = 15, it causes the current for
# compartment 2 to jump to a negative value.  The voltage, since it is
# integrating the current, begins to decrease.  The current curve is concave
# up since the current value is returning to 0 over time.

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

from nxsdk.arch.n2a.n2board import N2Board
import matplotlib.pyplot as plt
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays
# N2Board module provides access to the neuromorphic hardware

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
    numChips = 1
    # Number of cores per chip
    numCoresPerChip = [1]
    # Number of synapses per core
    numSynapsesPerCore = [[5]]
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
    n2Core.numUpdates.configure(numUpdates=1)
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
    # Connect compartment 0 to compartment 1 and 2
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
    targetSynapseLen = 2
    n2Core.synapseMap[targetAxonId].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[targetAxonId].synapseLen = targetSynapseLen
    # Set the synapseMap as a discreteMapEntry, i.e. simply maps to synapses.
    n2Core.synapseMap[targetAxonId].discreteMapEntry.configure()

    # Configure synapses

    # Excitatory
    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # Each core supports 15 different types of synapses, i.e. indices 1 to 15,
    # and index 0 is invalid. In this example synapse format ID 1 is being
    # used.
    n2Core.synapses[0].CIdx = 1
    n2Core.synapses[0].Wgt = 4
    n2Core.synapses[0].synFmtId = 1
    # Inhibitory
    # This synapse connects to compartment 2 (i.e. cxCfg[2]).
    n2Core.synapses[1].CIdx = 2
    n2Core.synapses[1].Wgt = -4
    n2Core.synapses[1].synFmtId = 1
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
    # idxBits set to 1 maps to 6 bits.  This is the bit width of the
    # compartment index value ("idx") stored in synapse_mem that maps to the
    # actual compartment index CIdx stored in each synapse.  The mapping is
    # CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    n2Core.synapseFmt[1].idxBits = 1
    # Selects how synapses are compressed. Compression setting of 3 means
    # dense. Other modes include sparse, run-length compression, zero-mask
    # compression, etc.
    n2Core.synapseFmt[1].compression = 3
    # fanoutType of 1 indicates mixed fanout, i.e. no shared sign bit (included
    # in each weight value)
    n2Core.synapseFmt[1].fanoutType = 1

    # Return the configured board
    return board


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    # Setup the network
    board = setupNetwork()
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    # cxState[i] contains the state of cxCfg[i].  The probe method taks a list
    # of compartment indices, i.e. [0, 1, 2] in this example, to obtain the
    # data for the chosen field, e.g. u or v.  The data for each compartment
    # index is returned as an element of a list.
    mon = board.monitor
    uProbes = mon.probe(n2Core.cxState, [0, 1, 2], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, 2], 'v')

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(30)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    # The explanation of the behavior is in the tutorial introduction above.
    # The plot for compartment 1 voltage appears to not always reach the
    # threshold of 640 when it spikes.  This is because when v crosses
    # the threshold during time step t, it gets reset immediately. Since the
    # monitor reads out the v value at the end of the time step, the
    # intermediate values are not visible from the plot.  Compartment 0 voltage
    # is more consistent because it is only driven by a bias. Compartment 1
    # receives time varying input and thus the last value below threshold is
    # always different and thus it appears to be spiking at different levels.
    fig = plt.figure(1002)
    k = 1
    for j in range(0, 3):
        plt.subplot(3, 2, k)
        uProbes[j].plot()
        plt.title('u'+str(j))
        k += 1
        plt.subplot(3, 2, k)
        vProbes[j].plot()
        plt.title('v'+str(j))
        k += 1
    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_02_fig1002.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
