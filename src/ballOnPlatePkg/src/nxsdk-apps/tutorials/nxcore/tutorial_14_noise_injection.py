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
# Tutorial: tutorial_14_noise_injection.py
# -----------------------------------------------------------------------------
# The goal of this tutorial is to show how to inject noise into current(u) and
# voltage(v) variable. This tutorial creates two compartments, one each in two
# different cores, and injects noise into u for compartment 0 of core 0 and into
# v for compartment 0 of core 1. In order to demonstrate noise injection, we are
# going to create probes to monitor u field of compartment 0 of core0 and v field of
# compartment 0 of core1.

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
    # There is just one dendriteSharedCfg register structure per core. So if
    # we set noiseAtDendOrVm to 1 in dendrtieSharedCfg of a core, noise will
    # be injected in v for all compartments having enableNoise 1 for that neuroCore.
    # Similarly if we set noiseAtDendOrVm to 0 in dendrtieSharedCfg of a core,
    # noise will be injected in u for all compartments of the neurocore having
    # enableNoise 1. Since we want to inject noise into both u nad v, we will need
    # two different cores with one compartment each to demonstrate this.

    numCoresPerChip = [2]
    # Number of synapses per core
    numSynapsesPerCore = [[5, 5]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only two in this example)
    n2Core0 = board.n2Chips[0].n2Cores[0]
    n2Core1 = board.n2Chips[0].n2Cores[1]

    # -------------------------------------------------------------------------
    # Configure two cores with one compartment each, having 0 bias current
    # and inject noise in u and v variable for compartment in core 0 and 1
    # respectively.
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[0] and vthProfileCfg[0] to
    # configure the compartment 0.
    # Voltage decay is set to 1/16 (2^12 factor for fixed point implementation)
    # Set enableNoise variable to 1, to enable noise injection.
    n2Core0.cxProfileCfg[0].configure(
        enableNoise=1, decayV=int(1 / 16 * 2 ** 12))
    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 10*2^6 = 640
    n2Core0.vthProfileCfg[0].staticCfg.configure(vth=10)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 0*(2^0) = 0.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core0.cxCfg[0].configure(
        bias=0,
        biasExp=0,
        vthProfile=0,
        cxProfile=0)

    # Doing the same configuration for Compartment 0 in n2Core1.
    n2Core1.cxProfileCfg[0].configure(
        enableNoise=1, decayV=int(1 / 16 * 2 ** 12))
    n2Core1.vthProfileCfg[0].staticCfg.configure(vth=5)
    n2Core1.cxCfg[0].configure(
        bias=0,
        biasExp=0,
        vthProfile=0,
        cxProfile=0)

    # Configure dendrite shared config to set the neuron noise.
    # To Configure noise injection in u or v following field needs to be set :
    # noiseExp0 : Exponent of Noise
    # noiseMantOffset0 : Mantissa of Noise
    # noiseAtDendOrVm : Decides whether the noise is to be injected in u or v.
    # For n2Core0 we will inject noise in u. So we will set noiseAtDendOrVm to 0.
    # The noise term is calculated by the following equation:
    # (lfsr() - 2**7 + noiseMantOffset0 * 2**6) * 2**(noiseExp0-7)
    # where lfsr() generates a random number which is then shifted and scaled.
    n2Core0.dendriteSharedCfg.configure(noiseExp0=2,
                                        noiseMantOffset0=1,
                                        noiseAtDendOrVm=0)

    # For n2Core1 we will inject noise in v. So we will set noiseAtDendOrVm to 1.
    n2Core1.dendriteSharedCfg.configure(noiseExp0=4,
                                        noiseMantOffset0=1,
                                        noiseAtDendOrVm=1)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0 are chosen for this example, cxMetaState[0] with phase0
    # is used. The setting of phase0 = PHASE_IDLE = 2 allows the compartment 0
    # to be driven by a bias current alone.
    # See user guide for all possible phases.
    n2Core0.cxMetaState[0].configure(phase0=2)
    # Similarly for n2Core1, i=0, so we will set cxMetaState[0], since
    # floor(0/4) = 0, and set phase0 = PHASE_IDLE = 2.
    n2Core1.cxMetaState[0].configure(phase0=2)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to compartments 0 to 4i-1 (i in range 1 to 256).
    n2Core0.numUpdates.configure(numUpdates=1)
    # Similarly for n2Core1
    n2Core1.numUpdates.configure(numUpdates=1)
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
    # Obtain the relevant core (only two in this example)
    n2Core0 = board.n2Chips[0].n2Cores[0]
    n2Core1 = board.n2Chips[0].n2Cores[1]

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    mon = board.monitor
    # cxState[i] contains the state of compartment[i]. Since we need to monitor
    # u,v values for Compartment 0 and 1, we need to setup four
    # probes : u0Probe and vProbe0 to monitor the value of u,v for compartment 0,
    # u1Probe and v1Probe for u,v of COmpartment 1.

    u0Probe = mon.probe(n2Core0.cxState, [0], 'u')[0]
    v0Probe = mon.probe(n2Core0.cxState, [0], 'v')[0]
    u1Probe = mon.probe(n2Core1.cxState, [0], 'u')[0]
    v1Probe = mon.probe(n2Core1.cxState, [0], 'v')[0]

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(100)
    board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1001)
    # Observation for Compartment 0 of Core 0 : Noise injected into u
    # Since there is no bias current, noise injected in u of Compartment 0 of
    # n2Core0 causes value of u to increase incrementally. Increase in value of
    # u causes the value of v to increment.The concave nature of the v curve
    # when the voltage is increasing is due to the decay constant.  Upon
    # reaching the threshold of 640, the compartment1 spikes and resets to 0.
    # Since there is no refractory period, the voltage immediate begins to
    # increase again.
    plt.subplot(2, 2, 1)
    u0Probe.plot()
    plt.title('u0')

    plt.subplot(2, 2, 2)
    v0Probe.plot()
    plt.title('v0')

    # Observation for Compartment 0 of Core 1 : Noise injected into v
    # Since there is no bias current, and no noise injected in u of Compartment 0,
    # value of u remains 0. Whereas since we are injecting voltage noise, the value
    # of v increases in correspondence to the noise. vth for Compartment 0 is 320 and
    # since it is never reached, it never gets reset and continues to increment.
    plt.subplot(2, 2, 3)
    u1Probe.plot()
    plt.title('u1')

    plt.subplot(2, 2, 4)
    v1Probe.plot()
    plt.title('v1')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_14_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
