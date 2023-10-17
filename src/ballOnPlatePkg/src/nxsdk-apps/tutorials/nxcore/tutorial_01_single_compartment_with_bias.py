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
# Tutorial: tutorial_01_single_compartment_with_bias.py
# -----------------------------------------------------------------------------
# This tutorial configures a single compartment and drives the voltage using
# only a bias current.  The current variable u takes a constant value of 0
# since there are no incoming spikes and noise is disabled.  The voltage
# variable v will grow due to the bias current and eventually spike when v
# crosses a threshold.  v resets upon spiking and, since no refractory
# period is configured, immediately begins to increase again.  The cycle
# repeats until the example completes.  Note that when v resets after spiking
# it does not reset to 0 since v is reused for tracking the refactory
# period.  Please see the tutorial on refactory delays for details.

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
    # Configure core with 1 compartment driven by bias only
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment prototype.  The compartment that is used is
    # also index 0, i.e. cxCfg[0].

    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12))
    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0 is used.
    # The setting of phase0 = PHASE_IDLE = 2 allows the compartment to be
    # driven by a bias current alone.  See user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 1000*2^6 = 64000
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=1000)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to comparments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^6) = 100*(2^6) = 6400.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=100,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

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
    # cxState[i] contains the state of cxCfg[i].  The probe method taks a list
    # of compartment indices, i.e. [0] in this example, to obtain the data for
    # the chosen field, e.g. u or v.  The data for each compartment index is
    # returned as an element of a list so in this example the only result is in
    # position 0.
    uProbe = mon.probe(n2Core.cxState, [0], 'u')[0]
    vProbe = mon.probe(n2Core.cxState, [0], 'v')[0]

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1001)
    # Since there are no incoming spikes and noise is disabled by default, u
    # remains constant at 0
    plt.subplot(1, 2, 1)
    uProbe.plot()
    plt.title('u')

    # v increases due to the bias current.  The concave nature of the curve
    # when the voltage is increasing is due to the decay constant.  Upon
    # reaching the threshold of 64000, the compartment spikes and resets to 0.
    # Since there is no refractory period, the voltage immediate begins to
    # increase again.
    plt.subplot(1, 2, 2)
    vProbe.plot()
    plt.title('v')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_01_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
