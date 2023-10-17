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
# Tutorial: tutorial_08_refractory_delays.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces refractory delays. We configure both static
# refractory delays and stochastic refractory delays. The stochastic
# refractory delays are accomplished by inserting noise into the refractory
# delay period. Four different neurons are configured with varying refractory
# delay periods. First, the simulation is run with static refractory delays,
# subsequently stochastic refractory delays are configured and run.
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


def setupNetwork(stochastic):

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
    numSynapsesPerCore = [[5]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with 4 different neurons driven by a bias current. Each
    # neuron has a different refractory delay
    # -----------------------------------------------------------------------

    # Here we are setting 4 different static refractory delays for each neuron
    if not stochastic:
        # Voltage decay (decayV) is set to 1/16 and the current decay (decayU)
        # is set to 1/10 (2^12 factor is for fixed point implementation)
        # Setting bapAction to 1 activates the refractory state

        # Setting refractDelay to 1
        n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         refractDelay=1)

        # Setting refractDelay to 3
        n2Core.cxProfileCfg[1].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         refractDelay=3)

        # Setting refractDelay to 8
        n2Core.cxProfileCfg[2].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         refractDelay=8)

        # Setting refractDelay to 15
        n2Core.cxProfileCfg[3].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         refractDelay=15)

    else:

        # Here we set 4 different stochastic refractory delay periods

        # Voltage decay (decayV) is set to 1/16 and the current delay (decayU)
        # is set to 1/10 (2^12 factor is for fixed point implementation)
        # Setting bapAction to 1 activates the refractory state
        # Setting threshOp to 1 enables spiking with random refractory delay

        # Setting refractory delay to 1
        n2Core.cxProfileCfg[0].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         threshOp=1,
                                         refractDelay=1)

        # Setting refractDelay to 3
        n2Core.cxProfileCfg[1].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         threshOp=1,
                                         refractDelay=3)
        # Setting refractDelay to 8
        n2Core.cxProfileCfg[2].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         threshOp=1,
                                         refractDelay=8)
        # Setting refractDelay to 15
        n2Core.cxProfileCfg[3].configure(decayV=int(1/16*2**12),
                                         decayU=int(1/10*2**12),
                                         bapAction=1,
                                         threshOp=1,
                                         refractDelay=15)

        # Configure dendrite shared config to set the neuron noise for refractory delays.
        # The noise term is calculated by the following equation:
        # (lfsr() - 2**7 + noiseMantOffset1 * 2**6) * 2**(noiseExp1-7)
        # where lfsr() generates a random number which is then shifted and scaled.
        n2Core.dendriteSharedCfg.configure(noiseExp1=2,
                                           noiseMantOffset1=1)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] is used.
    # There are 4 comparments per compartment group within one cxMetaState.
    # The 4 compartments are distinguished by phase0, phase1, phase2 and phase3.
    # The setting of the phases to 2 (PHASE_IDLE) allows the
    # compartment to be driven by an input spike.
    n2Core.cxMetaState[0].configure(phase0=2)
    n2Core.cxMetaState[0].configure(phase1=2)
    n2Core.cxMetaState[0].configure(phase2=2)
    n2Core.cxMetaState[0].configure(phase3=2)

    # Configure membrane potential threshold value mantissa.
    # Actual numerical threshold is vth*(2^6) = 10*2^6 = 640
    n2Core.vthProfileCfg[0].staticCfg.configure(vth=10)

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # Configure compartment specific bias mantissa and exponent.
    # Actual numerical bias is bias*(2^6) = 1*(2^6) = 64
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile  = i references cxProfileCfg[i].
    n2Core.cxCfg[0].configure(bias=1,
                              biasExp=6,
                              vthProfile=0,
                              cxProfile=0)

    n2Core.cxCfg[1].configure(bias=1,
                              biasExp=6,
                              vthProfile=0,
                              cxProfile=1)

    n2Core.cxCfg[2].configure(bias=1,
                              biasExp=6,
                              vthProfile=0,
                              cxProfile=2)

    n2Core.cxCfg[3].configure(bias=1,
                              biasExp=6,
                              vthProfile=0,
                              cxProfile=3)

    return board


if __name__ == "__main__":

    # First setup the network with static refractory delays
    stochastic = False

    # Setup the network
    board = setupNetwork(stochastic)

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
    uProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # Since the neurons are driven by bias and there are no incoming spikes
    # and no noise added to u, the u variable remains constant at 0 for all plots.

    # As the bias drives v the membrane threshold potential (in this case 640)
    # is met at which point the neuron spikes and the
    # refractory delay period (number of timesteps before v is reset and begins
    # accumulation again) will start. The same register used for variable v is
    # also leveraged to count the refractory delay: v = refractDly << 7 | refractCtr
    # where refractCtr is [1,refractDelay]. Therefore, once the neuron spikes the
    # v plot will jump due to refractDelay << 7 and then will increment by
    # 1 as the time steps progress. Once the refractory delay period finishes
    # v is reset to '0', again the refractDly is by default 1, v = 1 << 7 | 1 = 128
    # Therefore, the v plot will indicate 128 upon reset instead of 0.

    # Static refractory delay of 1 (i.e. no refractory delay)
    fig1 = plt.figure(11)
    plt.subplot(2, 1, 1)
    uProbes[0].plot()
    plt.title('First Neuron')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbes[0].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Static refractory delay of 3
    fig2 = plt.figure(12)
    plt.subplot(2, 1, 1)
    uProbes[1].plot()
    plt.title('Second Neuron')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbes[1].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Static refractory delay of 8
    fig3 = plt.figure(13)
    plt.subplot(2, 1, 1)
    uProbes[2].plot()
    plt.title('Third Neuron')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbes[2].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Static refractory delay of 15
    fig4 = plt.figure(14)
    plt.subplot(2, 1, 1)
    uProbes[3].plot()
    plt.title('Fourth Neuron')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbes[3].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Now run again with stochastic refractory delay periods
    stochastic = True

    # Setup the network
    board = setupNetwork(stochastic)

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
    uProbesRand = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'u')
    vProbesRand = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'v')

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # Here we see varying stochastic refractory delay periods in v, centered
    # around the refractory delay period set for each neuron with the injected
    # noise that was configured in the dendrite shared config.

    # Stochastic refractory delay 1
    fig5 = plt.figure(21)
    plt.subplot(2, 1, 1)
    uProbesRand[0].plot()
    plt.title('First Neuron stochastic refractory delay')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbesRand[0].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Stochastic refractory delay 3
    fig6 = plt.figure(22)
    plt.subplot(2, 1, 1)
    uProbesRand[1].plot()
    plt.title('Second Neuron stochastic refractory delay')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbesRand[1].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Stochastic refractory delay 8
    fig7 = plt.figure(23)
    plt.subplot(2, 1, 1)
    uProbesRand[2].plot()
    plt.title('Third Neuron stochastic refractory delay')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbesRand[2].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Stochastic refractory delay 15
    fig8 = plt.figure(24)
    plt.subplot(2, 1, 1)
    uProbesRand[3].plot()
    plt.title('Fourth Neuron stochastic refractory delay')
    plt.ylabel('Membrane current')
    plt.subplot(2, 1, 2)
    vProbesRand[3].plot()
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Show all plots from static refractory delays as well as
    # from stochastic refractory delays
    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_08_fig11.png"
        fileName2 = "tutorial_08_fig12.png"
        fileName3 = "tutorial_08_fig13.png"
        fileName4 = "tutorial_08_fig14.png"
        fileName5 = "tutorial_08_fig21.png"
        fileName6 = "tutorial_08_fig22.png"
        fileName7 = "tutorial_08_fig23.png"
        fileName8 = "tutorial_08_fig24.png"
        print("No display available, saving to files " + fileName1 + " through " +
              fileName4 + " and " + fileName5 + " through " + fileName8 + ".")
        fig1.savefig(fileName1)
        fig2.savefig(fileName2)
        fig3.savefig(fileName3)
        fig4.savefig(fileName4)
        fig5.savefig(fileName5)
        fig6.savefig(fileName6)
        fig7.savefig(fileName7)
        fig8.savefig(fileName8)
