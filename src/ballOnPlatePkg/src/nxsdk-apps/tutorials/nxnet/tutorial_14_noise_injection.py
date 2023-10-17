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
# finally returns the probes
# -----------------------------------------------------------------------------


def setupNetwork(net):

    # Create two compartment prototype with the following parameters:
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
    # randomizeCurrent: Inject noise in current(u)
    # enableNoise : enables the noise
    # The noise term is calculated by the following equation:
    # (lfsr() - 2**7 + noiseMantAtCompartment * 2**6) * 2**(noiseExpAtCompartment-7)
    # where lfsr() generates a random number which is then shifted and scaled.
    prototype1 = nx.CompartmentPrototype(biasMant=0,
                                         biasExp=0,
                                         vThMant=10,
                                         functionalState=2,
                                         compartmentVoltageDecay=256,
                                         compartmentCurrentDecay=1,
                                         logicalCoreId=0,
                                         enableNoise=1,
                                         randomizeCurrent=1,
                                         noiseMantAtCompartment=1,
                                         noiseExpAtCompartment=2
                                         )

    compartment1 = net.createCompartment(prototype1)

    # Creating prototype of second compartment in logicalCore 1 with paramter randomizeVoltage as 1
    # randomizeVoltage: Inject noise in voltage(v)
    prototype2 = nx.CompartmentPrototype(biasMant=0,
                                         biasExp=0,
                                         vThMant=5,
                                         functionalState=2,
                                         compartmentVoltageDecay=256,
                                         logicalCoreId=1,
                                         enableNoise=1,
                                         randomizeVoltage=1,
                                         noiseMantAtCompartment=1,
                                         noiseExpAtCompartment=4
                                         )

    compartment2 = net.createCompartment(prototype2)

    # Setup Probes
    # Creating probes to observe current and voltage of Compartment 1 and 2
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE]
    cx1Probe = compartment1.probe(probeParameters)
    cx2Probe = compartment2.probe(probeParameters)

    return (cx1Probe, cx2Probe)


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Create a network
    net = nx.NxNet()

    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    probes = setupNetwork(net)

    # -------------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------------
    net.run(100)
    net.disconnect()
    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    COMPARTMENT_CURRENT = 0
    COMPARTMENT_VOLTAGE = 1
    uProbes = [probe[COMPARTMENT_CURRENT] for probe in probes]
    vProbes = [probe[COMPARTMENT_VOLTAGE] for probe in probes]

    fig = plt.figure(1001)

    # Observation for Compartment 1 : Noise injected into u
    # Since there is no bias current, noise injected in u of Compartment 1
    # causes value of u to increase incrementally. Increase in value of
    # u causes the value of v to increment.The concave nature of the v curve
    # when the voltage is increasing is due to the decay constant.  Upon
    # reaching the threshold of 640, the Compartment1 spikes and resets to 0.
    # Since there is no refractory period, the voltage immediate begins to
    plt.subplot(2, 2, 1)
    uProbes[0].plot()
    plt.title('First Neuron')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    plt.subplot(2, 2, 2)
    vProbes[0].plot()
    plt.title('First Neuron')
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Observation for Compartment 2: Noise injected into v
    # Since there is no bias current, and no noise injected in u of Compartment 2,
    # value of u remains 0. Whereas since we are injecting voltage noise, the value
    # of v increases in correspondence to the noise. vth for Compartment 2 is 320 and
    # since it is never reached, it never gets reset and continues to increment.
    plt.subplot(2, 2, 3)
    uProbes[1].plot()
    plt.title('Second Neuron')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    plt.subplot(2, 2, 4)
    vProbes[1].plot()
    plt.title('Second Neuron')
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_14_compiled_noise_injection_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
