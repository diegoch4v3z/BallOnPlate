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
# Tutorial: tutorial_17_spike_probes.py
# -----------------------------------------------------------------------------
# This tutorial illustrates the use of spike probes to capture the occurence
# and count of spikes over specified intervals of time.

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

from nxsdk.graph.monitor.probes import *
from nxsdk.arch.n2a.n2board import N2Board
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
    # translates to comparments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 2*(2^6) = 128.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=2,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # -------------------------------------------------------------------------
    # Connect compartment 0 to compartment 1
    # -------------------------------------------------------------------------

    # Configure output axon
    targetCoreId = 4
    targetAxonId = 0
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(0, targetChipId, targetCoreId, targetAxonId)

    # Configure input axon

    # Setup the connection to the synapses.  The
    # targetSynapsePtr indexes into synapses and targetSynapseLen denotes how
    # many consecutive indices in synapses (starting from targetSynapsePtr) are
    # connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 1
    n2Core.synapseMap[targetAxonId].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[targetAxonId].synapseLen = targetSynapseLen
    # Set the synapseMap as a discreteMapEntry, i.e. simply maps to synapses.
    n2Core.synapseMap[targetAxonId].discreteMapEntry.configure()

    # Configure synapses

    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # Each core supports 15 different types of synapses, i.e. indices 1 to 15,
    # and index 0 is invalid. In this example synapse format ID 1 is being
    # used.
    n2Core.synapses[0].CIdx = 1
    n2Core.synapses[0].Wgt = 4
    n2Core.synapses[0].synFmtId = 1
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


if __name__ == "__main__":

    # ---------------------------------------------------------------------
    # Configure network
    # ---------------------------------------------------------------------
    # Setup the network
    board = setupNetwork()
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # ---------------------------------------------------------------------
    # Configure probes
    # ---------------------------------------------------------------------

    mon = board.monitor

    vProbes = mon.probe(n2Core.cxState, [0], 'v')

    # default
    # tStart = 1 (probe starts counting spikes at time step 1)
    # dt = 1 (probe counts the number of spikes in each time step)
    spikeProbes = mon.probe(n2Core.cxState, [0], 'spike')

    # larger dt
    # dt = 10 causes the probe to accumulate the number of ocurring spikes over 10 time steps
    customSpikeProbeCond = SpikeProbeCondition(dt=10, tStart=1)
    spikeProbesCust = mon.probe(
        n2Core.cxState, [0], 'spike', probeCondition=customSpikeProbeCond)

    # dt = 0
    # This setting never resets the counter in the probe so it counts and accumulates the number of spikes over the
    # entire duration of the run
    customSpikeProbeCond2 = SpikeProbeCondition(dt=0, tStart=1)
    spikeProbesCust2 = mon.probe(
        n2Core.cxState, [0], 'spike', probeCondition=customSpikeProbeCond2)

    # larger tStart, different dt
    # tStart = 15 means that the probe first begins reporting spike counts at time step 15.  Subsequently, the
    # spike counts are accumulated over 19 time steps (i.e. dt = 19)
    customSpikeProbeCond3 = SpikeProbeCondition(dt=19, tStart=15)
    spikeProbesCust3 = mon.probe(
        n2Core.cxState, [0], 'spike', probeCondition=customSpikeProbeCond3)

    # ---------------------------------------------------------------------
    # Run
    # ---------------------------------------------------------------------
    board.run(151)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure()

    plt.subplot(5, 1, 1)
    vProbes[0].plot()
    plt.title('v0')

    # A spike count is reported every time step.  It takes the value 1 when a spike occurs, i.e. seen when v resets,
    # and is 0 at other time steps.
    plt.subplot(5, 1, 2)
    spikeProbes[0].plot()
    plt.title('spikes (tStart=1, dt=1, default)')

    # A spike count is reported every 10 time steps and the value is the number of spikes in that 10 time step interval.
    plt.subplot(5, 1, 3)
    spikeProbesCust[0].plot()
    plt.title('spikes (tStart=1, dt=10)')

    # The spike count increments every time a spike occurs.  The counter is never reset.
    plt.subplot(5, 1, 4)
    spikeProbesCust2[0].plot()
    plt.title('spikes (tStart=1, dt=0)')

    # The spike count first reports a value of 2 at time step 15 since 2 spikes occurred.  Subsequently, the number of
    # spikes within 19 time steps is reported.
    plt.subplot(5, 1, 5)
    spikeProbesCust3[0].plot()
    plt.title('spikes (tStart=15, dt=19)')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_17_fig1.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
