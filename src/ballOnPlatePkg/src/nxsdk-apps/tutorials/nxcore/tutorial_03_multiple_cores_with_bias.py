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
# Tutorial: tutorial_03_multiple_cores_with_bias.py
# -----------------------------------------------------------------------------
# This tutorial configures 4 compartments in separate cores and drives the
# voltage using only a bias current on one. There exists a bias driven neuron
# on source core, and three other neurons on different cores of the same chip.
#
# Spikes are sent from the neuron on source core to all other neurons which
# have different threshold voltages. On the source compartment, the current
# variable u takes a constant value of 0 since there are no incoming spikes
# and noise is disabled. The voltage variable v will grow due to the bias
# current and eventually spike when v crosses a threshold. v resets upon
# spiking and, since no refractory period is configured, immediately begins
# to increase again. The cycle repeats until the example completes. Note that
# when v resets after spiking it does not reset to 0 since v is reused for
# tracking the refractory period. Please see the tutorial on refractory delays
# for details. The spike from one neuron is transmitted to other neurons.

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
    """Setup network"""

    # -------------------------------------------------------------------------
    # Initialize board
    # -------------------------------------------------------------------------

    # N2Board ID
    boardId = 1
    # Number of chips
    numChips = 1
    # Number of cores per chip
    numCoresPerChip = [4]
    # Number of synapses per core
    numSynapsesPerCore = [[1, 1, 1, 1]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant cores (four in total)
    n2Cores = board.n2Chips[0].n2CoresAsList

    # -------------------------------------------------------------------------
    # Configure one of the cores (n2Cores[0]) to be driven by bias
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartment that is used is
    # also index 0, i.e. cxCfg[0].

    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    n2Cores[0].cxProfileCfg[0].configure(decayV=int(1/16*2**12))
    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0 is used.
    # The setting of phase0 = PHASE_IDLE = 2 allows the compartment to be
    # driven by a bias current alone.  See user guide for all possible phases.
    n2Cores[0].cxMetaState[0].configure(phase0=2)
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 1000*2^6 = 64000
    n2Cores[0].vthProfileCfg[0].staticCfg.configure(vth=1000)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to comparments 0 to 4i-1 (i in range 1 to 256).
    n2Cores[0].numUpdates.configure(numUpdates=1)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^6) = 100*(2^6) = 6400.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Cores[0].cxCfg[0].configure(
        bias=100,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # -------------------------------------------------------------------------
    # Configure destination cores, synaptic and axon connectivity
    # -------------------------------------------------------------------------

    for coreId, n2Core in enumerate(n2Cores[1:]):
        # The parameter numUpdates controls how many compartment groups will be
        # serviced (4 compartments per compartment group). Specifically,
        # numUpdates = i means that i compartment groups will be update, which
        # translates to comparments 0 to 4i-1 (i in range 1 to 256).
        n2Core.numUpdates.configure(numUpdates=1)

        # Configuring threshold value mantissa.  Actual numerical threshold is
        # vth*(2^6) = 1000*2^6 = 64000. This is multiplied by the coreId to
        # simulate different threshold values for the three neurons.
        n2Core.vthProfileCfg[0].staticCfg.configure(vth=1000 * (coreId + 1))

        # Configure destination compartments for the only synapse. In the
        # following, synapse 0 is the connection to compartment CIdx 0.
        n2Core.synapses[0].CIdx = 0

        # Configure synaptic parameters, weight and format ID. Here,
        # for simplicity,we have kept all synaptic weights = 64. "Synapse
        # format ID" refers to the ID of a set of configurations that are
        # common to the synapses using that particular ID. Here, we have only
        # one type of synapses, whose shared properties are encapsulated in
        # synFmtId = 1 (see below).
        n2Core.synapses[0].Wgt = 64
        n2Core.synapses[0].synFmtId = 1

        # Configure the synapse format. Here, one may specify additional
        # tweaks to synaptic properties, such as scaling the weights using a
        # weight exponent, or bit-width of the weights, etc. A detailed
        # explanation of these is not within the scope of this tutorial.
        n2Core.synapseFmt[1].wgtExp = 0
        n2Core.synapseFmt[1].wgtBits = 7
        n2Core.synapseFmt[1].numSynapses = 63
        n2Core.synapseFmt[1].cIdxOffset = 0
        n2Core.synapseFmt[1].cIdxMult = 0
        n2Core.synapseFmt[1].idxBits = 1
        n2Core.synapseFmt[1].fanoutType = 2
        n2Core.synapseFmt[1].compression = 0

        # SynapseMap is the data structure that holds mapping from axons to
        # synapses. n2Core.synapseMap[j] corresponds to axonId = j.
        # synapsePtr = index of the first destination synapse
        # synapseLen = number of destination synapses spanned by axonId j (
        # axonal arbour of axonId j) discreteMapEntry.cxBase pertains to
        # counting the target compartment ID from a base offset = cxBase.
        # This is outside the scope of this tutorial, consult the detailed
        # documentation. But in this particular case, this entry needs to
        # be 0 for both synapseMap entries.

        n2Core.synapseMap[0].synapsePtr = 0
        n2Core.synapseMap[0].synapseLen = 1
        n2Core.synapseMap[0].discreteMapEntry.configure()

        # Configure output Axon from Source core 0 to three destination cores
        # We use createDiscreteAxon on the Source core and pass the
        # Source compartment, Destination chip ID (same here as it's a single
        # chip), Destination core and Destination Synapse Map ID
        n2Cores[0].createDiscreteAxon(srcCxId=0,
                                      dstChipId=0,
                                      dstCoreId=n2Core.id,
                                      dstSynMapId=0)

    # Return the configured board
    return board


# Helper function to generate a list of probes (u, v and spikes) for given
# cores.
# cxState[i] contains the state of cxCfg[i].  The probe method taks a list
# of compartment indices, i.e. [0] in this example, to obtain the data for
# the chosen field, e.g. u, v or spike.  The data for each compartment index is
# returned as an element of a list so in this example the only result is in
# position 0.

def generateProbes(mon, n2Cores):
    """
    Return a probe generator
    :param mon: Monitor associated with the board
    :param n2Cores: List of cores to collect probes for
    :return: Generator with u, v and spike Probe for each core
    """
    for n2Core in n2Cores:
        yield (mon.probe(n2Core.cxState, [0], 'u')[0],
               mon.probe(n2Core.cxState, [0], 'v')[0],
               mon.probe(n2Core.cxState, [0], 'spike')[0])


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    # Setup the network
    board = setupNetwork()
    # Obtain the relevant cores
    n2Cores = board.n2Chips[0].n2CoresAsList

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    mon = board.monitor

    (uProbes, vProbes, spikeProbes) = zip(*list(generateProbes(mon, n2Cores)))

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1001)
    figIndex = iter(range(1, 13))

    for coreId, (uProbe, vProbe, spikeProbe) in enumerate(
            zip(uProbes, vProbes, spikeProbes)):
        # Since there are no incoming spikes and noise is disabled by default, u
        # remains constant at 0 for source core
        plt.subplot(4, 3, next(figIndex))
        uProbe.plot()
        plt.title('%d: u' % coreId)

        # v increases due to the bias current.  The concave nature of the curve
        # when the voltage is increasing is due to the decay constant.  Upon
        # reaching the threshold of 64000, the compartment spikes and resets
        # to 0. Since there is no refractory period, the voltage immediate
        # begins to increase again.
        plt.subplot(4, 3, next(figIndex))
        vProbe.plot()
        plt.title('%d: v' % coreId)

        plt.subplot(4, 3, next(figIndex))
        spikeProbe.plot()
        plt.title('%d: spike' % coreId)

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_19_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
