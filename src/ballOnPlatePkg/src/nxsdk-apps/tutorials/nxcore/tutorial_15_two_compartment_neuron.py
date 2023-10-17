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
# Tutorial: tutorial_15_two_compartment_neuron.py
# -----------------------------------------------------------------------------
# The goal of this tutorial is to show an example of how to create a multi-
# compartment neuron and demonstrate basic principle how information
# flows from one compartment via stack to another one. In this tutorial we will
# create a two compartment neuron by configuring stackOut and stackInt and using
# basic ADD join function.
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
    numSynapsesPerCore = [[1]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure core with 2 compartments driven by bias only
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment 0.

    # Voltage decay is set to 0
    # stackOut is set to 1 to push the value of v onto the Stack.
    n2Core.cxProfileCfg[0].configure(decayV=0, stackOut=1)
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 1000*2^6 = 64000
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=1000)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 100*(2^0) = 100.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=100,
        biasExp=0,
        vthProfile=0,
        cxProfile=0)

    # Voltage decay is set to 0
    # stackIn is set to 2 to POP the value from stack. Since we are pushing
    # the value of v of compartment 0 on the stack, it will be poped.
    # joinOp is set to 1, which means ADD will be used for Join Operation
    # ADD will add the poped value from the stack to the current v value
    # For other supported Join Operations see user guide.
    n2Core.cxProfileCfg[1].configure(decayV=0, stackIn=2, joinOp=1)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(biasExp) = 0*(2^0) = 0.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 1 references cxProfileCfg[1].
    n2Core.cxCfg[1].configure(
        bias=0,
        biasExp=0,
        vthProfile=0,
        cxProfile=1)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0,1 are chosen for this example, cxMetaState[0] with phase0
    # and phase1 is used. The setting of phase0 = phase1 = PHASE_IDLE = 2
    # allows the compartment 0 and 1 to be driven by a bias current alone.
    # See user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2, phase1=2)
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
    # Observation for Compartment 0
    # v increases due to the bias current.  The linear nature of the plot
    # is due to the 0 decay constant. Since it doesnot reach threshold,
    #  it never gets reset.
    plt.subplot(2, 1, 1)
    v0Probe.plot()
    plt.title('v0')

    # Observation for Compartment 1
    # There is no bias current, so the value of v is solely driven by aggregation
    # of the voltage value from the Compartment 0.
    # V(Comp1) = V(Comp1) + V(Comp 0)
    # V(Comp 0) is being pushed on the stack by Compartment 0 using StackIn operation and
    # popped by the StackOut operation, Joined using joinOp Add function.
    plt.subplot(2, 1, 2)
    v1Probe.plot()
    plt.title('v1')
    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_15_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
