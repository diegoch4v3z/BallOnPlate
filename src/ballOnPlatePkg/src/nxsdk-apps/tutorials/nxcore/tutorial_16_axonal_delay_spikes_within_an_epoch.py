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
# Tutorial: tutorial_16_axonal_delay_spikes_within_an_epoch.py
# -----------------------------------------------------------------------------
#
# This tutorial illustrates the impact of the tepoch setting when axonal_delay
# is enabled.  We configure two independent compartments each connected to a
# separate compartment through a synapse. We set the same axonal delay for each
# compartment. For the first pair of compartments, the v threshold is set low
# enough such that two spikes are generated within tepoch.  The second spike
# is thus lost and is not sent to the connected compartment.  The second pair
# of compartments enable refractory delay and thus eliminate the occurrence of
# a second spike within tepoch.  In general, refractory delay is recommended
# to be set to tepoch.

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
    numSynapsesPerCore = [[4]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure core with two neurons connected to two compartments driven by
    # input spikes
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0], cxProfileCfg[1] and
    # vthProfileCfg[0] to configure the compartment prototypes.  The
    # compartments that are used are indices 0, 1, 2, 3 i.e. cxCfg[0],
    # cxCfg[1], cxCfg[2] and cxCfg[3]. The compartments will share the same
    # vthProfileCfg.  Compartments 0 and 1 use cxProfileCfg[0] and compartments
    # 2 and 3 us cxProfileCfg[1].

    n2Core.cxCfg[0].configure(vthProfile=0,
                              cxProfile=0)

    n2Core.cxCfg[1].configure(vthProfile=0,
                              cxProfile=0)

    n2Core.cxCfg[2].configure(vthProfile=0,
                              cxProfile=1)

    n2Core.cxCfg[3].configure(vthProfile=0,
                              cxProfile=1)

    # The current decay (decayU) is set to 1/10 (2^12 factor is for
    # fixed point implementation)
    # Set enableAxonDelay to activate axonal delay
    n2Core.cxProfileCfg[0].configure(decayU=int(1/10*2**12),
                                     enableAxonDelay=1)

    # Enable refractory delay as well
    n2Core.cxProfileCfg[1].configure(decayU=int(1/10*2**12),
                                     enableAxonDelay=1,
                                     bapAction=1,
                                     refractDelay=8)

    # Configure axonal delay of 3 for neuron 0 and 2
    n2Core.somaState[0].configure(axonDelay=3)
    n2Core.somaState[2].configure(axonDelay=3)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # The setting of phase0 = PHASE_IDLE = 2 allows the
    # compartment to be driven by an input spike. See the user guide for all
    # possible phases.
    n2Core.cxMetaState[0].configure(phase0=2,
                                    phase1=2,
                                    phase2=2,
                                    phase3=2)

    # Configure threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 20*2^6 = 1280
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=20)

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # Configure tepoch to be at least a third of the axonDelay,
    # i.e. tepoch*3 >= axonDelay. This is due to how the axon delay
    # is implemented in HW using a queue to track the axon delay.
    n2Core.dendriteTimeState.configure(tepoch=8)

    # Configure output axon
    targetCoreId = 4
    targetAxonId = 1
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(0, targetChipId, targetCoreId, targetAxonId)

    targetCoreId = 4
    targetAxonId = 3
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(2, targetChipId, targetCoreId, targetAxonId)

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    for loop in range(0, 4):
        n2Core.synapseMap[loop].synapsePtr = loop
        n2Core.synapseMap[loop].synapseLen = 1
        n2Core.synapseMap[loop].discreteMapEntry.configure()

    # This synapse connects to compartment 0 (i.e. cxCfg[0]).
    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits. It is a normalized signed value.
    # Here, the synaptic weight of 10 is used.
    # Each core supports 15 different types of synapse formats,
    # i.e. indices 1 to 15 where 0 is invalid. We create a single snyFmt
    # and use ID 1.
    for loop in range(0, 4):
        n2Core.synapses[loop].CIdx = loop
        n2Core.synapses[loop].Wgt = 10
        n2Core.synapses[loop].synFmtId = 1

    # Configure synaptic format

    # The wgtBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of 7 maps to 8 bits, thus keeping
    # the full precision of the synaptic weight value.
    # Note, if the synapseFmt[i].fanoutType is 1 (mixed), then the sign bit
    # is included in the weight value, thereby losing one more LSB.
    n2Core.synapseFmt[1].wgtBits = 7

    # Setting numSynapses specifies a certain form of encoding.
    # The value of 1 means there is 1 synapse and the user has specified
    # the mapping. The value of 63 is a special value and does not mean 63
    # synapses, but instead is a special value to have
    # the number of synapses encoded in the prefix field of synapse_mem.
    # In other words, using 63 indicates to the compiler to figure out how
    # to map synapses into synapse entries, where as setting a different
    # value for numSynapses indicates that the user specifies the mapping.
    n2Core.synapseFmt[1].numSynapses = 63

    # The idxBits field is the bit width of the compartment index value
    # ("idx") stored in synapse_mem that maps to the actual compartment
    # index CIdx stored in each synapse. CIdx is calculated by the
    # following: CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    # Valid values for idxBits are 0..7 which map
    # to 0,6,7,8,9,10,11,12. Here, idxBits of 5 maps to 10.
    n2Core.synapseFmt[1].idxBits = 5

    # Compression indicates how the synapses are compressed. Here we
    # select dense (shared index) uncompressed.
    # 0: sparse
    # 3: dense (shared index) uncompressed
    n2Core.synapseFmt[1].compression = 0

    # The fanoutType controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    n2Core.synapseFmt[1].fanoutType = 1

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

    # Generate an input spike at timestep 3 to chip 0, core 4 and axon 2
    axonId = 2
    bs.addSpike(timestep, chipId, coreId, axonId)

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    # cxState[i] contains the state of cxCfg[i].  The probe method takes
    # a list of compartment indices to obtain the data for the
    # chosen field, e.g. u or v.  The data for each compartment index
    # is returned as an element of a list.
    uProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    # Compartment 0 receives the input spike at time step 3 at which point u0
    # increases
    fig = plt.figure(1002)
    plt.subplot(4, 2, 1)
    uProbes[0].plot()
    plt.title('u0')
    plt.subplot(4, 2, 2)
    vProbes[0].plot()
    plt.title('v0')

    # Compartment 0 has an axonal delay of 3 and is connected to compartment 1.
    # u1 accumulates at time step 9. The spike arrived at compartment 0 at time
    # step 3, compartment 0 spikes at time step 5 and delayed for time steps 6,
    # 7, and 8. Spike is sent to compartment 1 on time step 9 to be accumulated
    # in u1 at that time step.
    #
    # Note that v0 also spikes again at time step 9.  However, because tepoch
    # set to 8, this second spike is not sent to compartment 1.  The second
    # jump in u1 occurs at time step 19, which is corresponding to a spike in
    # v0 at time step 15.
    plt.subplot(4, 2, 3)
    uProbes[1].plot()
    plt.title('u1')
    plt.subplot(4, 2, 4)
    vProbes[1].plot()
    plt.title('v1')

    # Compartment 2 receives the input spike at time step 3 at which point u2
    # increases
    plt.subplot(4, 2, 5)
    uProbes[2].plot()
    plt.title('u2')
    plt.subplot(4, 2, 6)
    vProbes[2].plot()
    plt.title('v2')

    # Compartment 2 has an axonal delay of 3 and is connected to compartment 3.
    # When v2 spikes at time step 5, it is followed by a refractory delay of 8
    # time steps (i.e. time steps 5 through 12). On time step 13, v2 begins to
    # accumulate once more.  Due to the refractory delay, no spikes are lost
    # as the two spikes generated by v2 crossing threshold are reflected in u3.
    plt.subplot(4, 2, 7)
    uProbes[3].plot()
    plt.title('u3')
    plt.subplot(4, 2, 8)
    vProbes[3].plot()
    plt.title('v3')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_16_fig1002.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
