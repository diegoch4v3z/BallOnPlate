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


def setupNetwork(net):

    # Create a compartment prototype with the following parameters:
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
    prototype1 = nx.CompartmentPrototype(biasMant=100,
                                         biasExp=6,
                                         vThMant=1000,
                                         functionalState=2,
                                         compartmentVoltageDecay=256,
                                         compartmentCurrentDecay=0,
                                         logicalCoreId=0)

    compartment1 = net.createCompartment(prototype1)

    # Specify the compartment states that we want to probe - i.e. "probeParameters"
    # Below we want to probe the compartment current(U), compartment voltage(V) and the spikes produced by the compartment
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                       nx.ProbeParameter.SPIKE]
    # Use default probe conditions - i.e. probeConditions=None
    probeConditions = None
    # Create a compartment probe to probe the states of each compartment as specified by the probeParameters and probeConditions
    cxProbes = compartment1.probe(probeParameters, probeConditions)

    # Store cxProbe and future compartment probe objects in a list
    probes = [cxProbes]

    # Create three more compartments on different cores and connect them to
    # the source compartment
    for logicalCore in range(1, 4):
        # vThMant: Configuring voltage threshold value mantissa for each
        # compartment. Actual numerical threshold equals vThMant*(2^6). vthMant
        # is scaled for each compartment to showcase differing spiking
        # thresholds. Effective threshold for each compartment is 1000 *
        # logicalCore * 2^6. For instance, for logicalCore=2, vthMant would
        # be 1000*2=2000 and the numeric threshold would be 2000*2^6=128000
        #
        # logicalCoreId: Initialize the compartment in logical core compartment
        compartmentPrototype = nx.CompartmentPrototype(
            vThMant=1000 * logicalCore,
            logicalCoreId=logicalCore,
            compartmentCurrentDecay=1)
        compartment = net.createCompartment(compartmentPrototype)
        connectionPrototype = nx.ConnectionPrototype(weight=64)
        net._createConnection(compartment1, compartment, connectionPrototype)

        # Create a compartment probe to probe the states of each compartment as specified by the probeParameters and probeConditions
        cxProbes = compartment.probe(probeParameters, probeConditions)
        probes.append(cxProbes)

    return probes


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
    # -------------------------------------------------------------------------

    net.run(100)
    net.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1003)
    figIndex = iter(range(1, 13))

    for coreId, cxProbes in enumerate(probes):
        # Since there are no incoming spikes and noise is disabled by default, u
        # remains constant at 0 for source core
        plt.subplot(4, 3, next(figIndex))
        uProbe = cxProbes[0]
        uProbe.plot()
        plt.title('%d: u' % coreId)

        # v increases due to the bias current.  The concave nature of the curve
        # when the voltage is increasing is due to the decay constant.  Upon
        # reaching the threshold of 64000, the compartment spikes and resets
        # to 0. Since there is no refractory period, the voltage immediate
        # begins to increase again.
        plt.subplot(4, 3, next(figIndex))
        vProbe = cxProbes[1]
        vProbe.plot()
        plt.title('%d: v' % coreId)

        plt.subplot(4, 3, next(figIndex))
        spikeProbe = cxProbes[2]
        spikeProbe.plot()
        plt.title('%d: spike' % coreId)

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_03_fig1003.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
