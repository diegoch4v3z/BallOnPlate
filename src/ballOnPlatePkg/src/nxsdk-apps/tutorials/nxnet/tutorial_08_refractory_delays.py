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
# This tutorial introduces refractory delays. We configure both static
# refractory delays and stochastic refractory delays. The stochastic
# refractory delays are accomplished by inserting noise into the refractory
# delay period. Four different neurons are configured with varying refractory
# delay periods. First, the simulation is run with static refractory delays,
# subsequently stochastic refractory delays are configured and run.

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

# For plotting without GUI
import matplotlib.pyplot as plt
import nxsdk.api.n2a as nx
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API

# plt is used for graphical displays

# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function defines an example NxNet network, uses N2Compiler to compile
# and initialize the software mappings to hardware components and
# finally returns the board
# -----------------------------------------------------------------------------


def setupNetwork(net, stochastic):

    # -----------------------------------------------------------------------
    # Configure 4 different neurons driven by a bias current. Each
    # neuron has a different refractory delay
    # -----------------------------------------------------------------------

    # Here we are setting 4 different static refractory delays for each neuron
    # Create four compartment prototypes with the following parameters:
    # biasMant: Configure bias mantissa.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 100*(2^6) = 6400.
    # biasExp: Configure bias exponent.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 100*(2^6) = 6400.
    # vThMant: Configuring voltage threshold value mantissa. Actual numerical
    #   threshold is vThMant*(2^6) = 1000*2^6 = 64000
    # functionalState: The setting to PHASE_IDLE = 2 allows the compartment to
    #   be driven by a bias current alone.  See user guide for all possible
    #   phases.
    # compartmentVoltageDecay: Set to 1/16 (2^12 factor for fixed point
    #   implementation) {1/16 * 2^12=256}
    # logicalCoreId: Initialize the compartment in logical core 0
    # compartmentCurrentDecay: Set to 1/10 (2^12 factor for fixed point
    #   implementation) {1/10 * 2^12}
    # refractoryDelay : Value of the refractory delay
    # Setting refractDelay to 1
    prototype = nx.CompartmentPrototype(biasMant=1,
                                        biasExp=6,
                                        vThMant=10,
                                        functionalState=2,
                                        compartmentVoltageDecay=256,
                                        logicalCoreId=0,
                                        compartmentCurrentDecay=int(
                                            1/10*2**12),
                                        refractoryDelay=1
                                        )

    # For stochastic refractory delay periods
    if stochastic:
        # Setting threshholdBehavior to 1 enables spiking with random refractory delay
        # Configure noiseExpAtRefractoryDelay and noiseMantAtRefractoryDelay
        # to set the neuron noise for refractory delays.
        # The noise term is calculated by the following equation:
        # (lfsr() - 2**7 + noiseMantAtRefractoryDelay * 2**6) * 2**(noiseExpAtRefractoryDelay-7)
        # where lfsr() generates a random number which is then shifted and scaled.
        prototype.thresholdBehavior = 1
        prototype.noiseExpAtRefractoryDelay = 2
        prototype.noiseMantAtRefractoryDelay = 1

    compartment1 = net.createCompartment(prototype)

    # Setting refractDelay to 3
    prototype.refractoryDelay = 3
    compartment2 = net.createCompartment(prototype)

    # Setting refractDelay to 8
    prototype.refractoryDelay = 8
    compartment3 = net.createCompartment(prototype)

    # Setting refractDelay to 15
    prototype.refractoryDelay = 15
    compartment4 = net.createCompartment(prototype)

    # Create Probes
    # Creating probes to observer Current and Voltage for all 4 compartments
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE]
    cxProbe1 = compartment1.probe(probeParameters)
    cxProbe2 = compartment2.probe(probeParameters)
    cxProbe3 = compartment3.probe(probeParameters)
    cxProbe4 = compartment4.probe(probeParameters)

    return (cxProbe1, cxProbe2, cxProbe3, cxProbe4)


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Create a network without stochastic delay
    with nx.NxNet() as net:

        # First setup the network with static refractory delays
        # -------------------------------------------------------------------------
        # Configure network
        # -------------------------------------------------------------------------
        stochastic = False
        probes = setupNetwork(net, stochastic)

        # -------------------------------------------------------------------------
        # Run
        # -------------------------------------------------------------------------

        net.run(100)

        # -------------------------------------------------------------------------
        # Plot
        # -------------------------------------------------------------------------
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

        COMPARTMENT_CURRENT = 0
        COMPARTMENT_VOLTAGE = 1
        uProbes = [probe[COMPARTMENT_CURRENT] for probe in probes]
        vProbes = [probe[COMPARTMENT_VOLTAGE] for probe in probes]

        # Static refractory delay of 1
        fig1 = plt.figure(11)
        plt.subplot(2, 1, 1)
        uProbes[0].plot()
        plt.title('First Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[0].plot()
        plt.title('First Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Static refractory delay of 3
        fig2 = plt.figure(12)
        plt.subplot(2, 1, 1)
        uProbes[1].plot()
        plt.title('Second Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[1].plot()
        plt.title('Second Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Static refractory delay of 8
        fig3 = plt.figure(13)
        plt.subplot(2, 1, 1)
        uProbes[2].plot()
        plt.title('Third Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[2].plot()
        plt.title('Third Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Static refractory delay of 15
        fig4 = plt.figure(14)
        plt.subplot(2, 1, 1)
        uProbes[3].plot()
        plt.title('Fourth Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[3].plot()
        plt.title('Fourth Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

    # Now run again with stochastic refractory delay periods

    # Create a network without stochastic delay
    with nx.NxNet() as net:

        # -------------------------------------------------------------------------
        # Configure network
        # -------------------------------------------------------------------------
        stochastic = True
        probes = setupNetwork(net, stochastic)

        # -------------------------------------------------------------------------
        # Run
        # -------------------------------------------------------------------------

        net.run(100)

        # -------------------------------------------------------------------------
        # Plot
        # -------------------------------------------------------------------------
        # Here we see varying stochastic refractory delay periods in v, centered
        # around the refractory delay period set for each neuron with the injected
        # noise that was configured in the dendrite shared config.

        COMPARTMENT_CURRENT = 0
        COMPARTMENT_VOLTAGE = 1
        uProbes = [probe[COMPARTMENT_CURRENT] for probe in probes]
        vProbes = [probe[COMPARTMENT_VOLTAGE] for probe in probes]

        # Stochastic refractory delay 1
        fig5 = plt.figure(21)
        plt.subplot(2, 1, 1)
        uProbes[0].plot()
        plt.title('First Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[0].plot()
        plt.title('First Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Stochastic refractory delay 3
        fig6 = plt.figure(22)
        plt.subplot(2, 1, 1)
        uProbes[1].plot()
        plt.title('Second Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[1].plot()
        plt.title('Second Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Stochastic refractory delay 8
        fig7 = plt.figure(23)
        plt.subplot(2, 1, 1)
        uProbes[2].plot()
        plt.title('Third Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[2].plot()
        plt.title('Third Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

        # Stochastic refractory delay 15
        fig8 = plt.figure(24)
        plt.subplot(2, 1, 1)
        uProbes[3].plot()
        plt.title('Fourth Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane current')

        plt.subplot(2, 1, 2)
        vProbes[3].plot()
        plt.title('Fourth Neuron')
        plt.xlabel('Time')
        plt.ylabel('Membrane voltage')

    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_08_compiled_fig11.png"
        fileName2 = "tutorial_08_compiled_fig12.png"
        fileName3 = "tutorial_08_compiled_fig13.png"
        fileName4 = "tutorial_08_compiled_fig14.png"
        fileName5 = "tutorial_08_compiled_fig21.png"
        fileName6 = "tutorial_08_compiled_fig22.png"
        fileName7 = "tutorial_08_compiled_fig23.png"
        fileName8 = "tutorial_08_compiled_fig24.png"
        print("No display available, saving to files "
              + fileName1 + " through " + fileName4 +
              " and " + fileName5 + " through " + fileName8 + ".")
        fig1.savefig(fileName1)
        fig2.savefig(fileName2)
        fig3.savefig(fileName3)
        fig4.savefig(fileName4)
        fig5.savefig(fileName5)
        fig6.savefig(fileName6)
        fig7.savefig(fileName7)
        fig8.savefig(fileName8)
