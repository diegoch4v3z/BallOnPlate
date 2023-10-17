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
# Tutorial: tutorial_10_membrane_potential_threshold.py
# -----------------------------------------------------------------------------
# The goal of this tutorial is to show two different ways of configuring vth for
# compartments. This tutorial configures two compartments, for compartment 0 vth
# value is configured using vthProfileCfg and for compartment 1 the vth value is
# configured using someStates nodeset.
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

    # N2Board ID
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
    # Configure core with 2 compartments driven by bias only
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment 0.

    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1 / 16 * 2 ** 12))
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 1000*2^6 = 64000
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=1000)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 100*(2^6) = 6400.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=100,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # The configuration below uses cxProfileCfg[1] and vthProfileCfg[1] to
    # configure the compartment 1. In vthProfileCfg[1] useSomaVth is set to 1, to
    # indicate the neurocore to use the value of vth set in somaState as threshold
    # for that compartment.
    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    n2Core.cxProfileCfg[1].configure(decayV=int(1 / 16 * 2 ** 12))
    # Configuring useSomaVth field in vthProfileCfg to use vth in somaState
    # as threshold.
    n2Core.vthProfileCfg[1].staticCfg.configure(useSomaVth=1)
    # Setting the vth value in somaState for that compartment.
    # Actual numerical threshold will be : vth * 2**6 = 500*64 = 32000
    n2Core.somaState[1].configure(vth=500)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 100*(2^6) = 6400.
    # vthProfile = 1 references vthProfileCfg[1].
    # cxProfile = 1 references cxProfileCfg[1].
    n2Core.cxCfg[1].configure(
        bias=100,
        biasExp=6,
        vthProfile=1,
        cxProfile=1)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] ,with the field phaseK where K = mod(i,4)
    # and field somaOpK where K = mod(i,4).
    # Since i=0,1 are chosen for this example, cxMetaState[0] with phase0
    # and phase1 is used. The setting of phase0 = phase1 = PHASE_IDLE = 2
    # allows the compartment 0 and 1 to be driven by a bias current alone.
    # To read vth value for compartment 1 from somaState, somaOp1 field needs to
    # be set to 3, which will do unconditional read of SomaState for somaState[1].
    # See user guide for all possible phases and somaOp.
    n2Core.cxMetaState[0].configure(phase0=2, phase1=2, somaOp1=3)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to compartments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)
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

    mon = board.monitor
    # cxState[i] contains the state of compartment[i]. Since we need to monitor
    # v values for Compartment 0 and 1, we need to setup two probes : v0Probe to
    # monitor the value of v for compartment 0 and v1Probe to monitor the v value
    # of compartment 1.

    v0Probe = mon.probe(n2Core.cxState, [0], 'v')[0]
    v1Probe = mon.probe(n2Core.cxState, [1], 'v')[0]

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1001)
    # v increases due to the bias current.  The concave nature of the curve
    # when the voltage is increasing is due to the decay constant.  Upon
    # reaching the threshold of 64000, the compartment1 spikes and resets to 0.
    # Since there is no refractory period, the voltage immediate begins to
    # increase again.
    plt.subplot(1, 2, 1)
    v0Probe.plot()
    plt.title('v0')

    # v for compartment 1 also demonstrates the same nature. But since vth for
    # it is 32000, it will get reset at 32000.
    plt.subplot(1, 2, 2)
    v1Probe.plot()
    plt.title('v1')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_10_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
