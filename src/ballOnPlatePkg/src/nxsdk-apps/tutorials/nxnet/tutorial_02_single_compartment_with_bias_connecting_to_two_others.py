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
# Tutorial:
# tutorial_02_single_compartment_with_bias_connecting_to_two_others.py
# -----------------------------------------------------------------------------
# This tutorial configures a single compartment and drives the voltage using
# only a bias current.  This compartment is then connected to two additional
# compartments, one through an excitatory synapse and one through an
# inhibitory synapse demonstrated via weights on the connections.
#
# Bias current driven compartment, i.e. compartment1:
# The behavior of the bias current driven compartment is described in tutorial
# 01.
#
# Compartment connected via excitatory synapse, i.e. compartment2:
# When compartment1 spikes at around t = 15, it causes the current for
# compartment2 to jump.  This results in the voltage for compartment2
# to increase and then spike.  The voltage then immediately begins to increase
# again since the current is still positive, though slowly decreasing.  The
# voltage reaches threshold and spikes again and the voltage again starts to
# increase.  Note that the rate of voltage increase after the second spike
# is slower than that after the first spike since the current value is
# decreasing.
#
# Compartment connected via inhibitory synapse, i.e. compartment3:
# When compartment1 spikes at around t = 15, it causes the current for
# compartment3 to jump to a negative value.  The voltage, since it is
# integrating the current, begins to decrease.  The current curve is concave
# up since the current value is returning to 0 over time.

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
    #   biasMant*(2^biasExp) = 1*(2^6) = 64.
    # biasExp: Configure bias exponent.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 1*(2^6) = 64.
    # vThMant: Configuring voltage threshold value mantissa. Actual numerical
    #   threshold is vThMant*(2^6) = 10*2^6 = 640
    # functionalState: The setting to PHASE_IDLE = 2 allows the compartment to
    #   be driven by a bias current alone.  See user guide for all possible
    #   phases.
    # compartmentVoltageDecay: Set to 1/16 (2^12 factor for fixed point
    #   implementation) {1/16 * 2^12=256}
    # compartmentCurrentDecay: Set to 1/10 (2^12 factor for fixed point
    #   implementation) {1/10 * 2^12=409}
    prototype1 = nx.CompartmentPrototype(biasMant=1,
                                         biasExp=6,
                                         vThMant=10,
                                         functionalState=2,
                                         compartmentVoltageDecay=256,
                                         compartmentCurrentDecay=410)

    compartment1 = net.createCompartment(prototype1)

    prototype2 = nx.CompartmentPrototype(vThMant=10,
                                         compartmentVoltageDecay=256,
                                         compartmentCurrentDecay=410)

    compartment2 = net.createCompartment(prototype2)
    compartment3 = net.createCompartment(prototype2)

    # Excitatory synapse (connection)
    connProto1 = nx.ConnectionPrototype(weight=4, compressionMode=3)
    net._createConnection(compartment1, compartment2, connProto1)

    # Inhibitatory synapse (connection)
    connProto2 = nx.ConnectionPrototype(weight=-4, compressionMode=3)
    net._createConnection(compartment1, compartment3, connProto2)

    # Specify the compartment states that we want to probe - i.e. "probeParameters"
    # Below we want to probe the compartment current(U) and compartment voltage(V)
    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                       nx.ProbeParameter.COMPARTMENT_VOLTAGE]
    # Use default probe conditions - i.e. probeConditions=None
    probeConditions = None
    # Create a compartment probe to probe the states of compartment1
    cx1Probes = compartment1.probe(probeParameters, probeConditions)
    # Create a compartment probe to probe the states of compartment2
    cx2Probes = compartment2.probe(probeParameters, probeConditions)
    # Create a compartment probe to probe the states of compartment3
    cx3Probes = compartment3.probe(probeParameters, probeConditions)

    # cx1Probes is a list of probe objects for compartment1.
    # cx1Probes[0] is the compartment voltage probe and cx1Probes[1] is the compartment current respectively

    # uProbes below is the list of all the compartment current probes
    uProbes = [cx1Probes[0], cx2Probes[0], cx3Probes[0]]
    # vProbes below is the list of all the compartment voltage probes
    vProbes = [cx1Probes[1], cx2Probes[1], cx3Probes[1]]

    # Return the configured probes
    return uProbes, vProbes


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Create a network
    net = nx.NxNet()

    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    uProbes, vProbes = setupNetwork(net)

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    net.run(30)
    net.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    # The explanation of the behavior is in the tutorial introduction above.
    # The plot for compartment2 voltage appears to not always reach the
    # threshold of 640 when it spikes.  This is because when v crosses
    # the threshold during time step t, it gets reset immediately. Since the
    # monitor reads out the v value at the end of the time step, the
    # intermediate values are not visible from the plot.  Compartment1 voltage
    # is more consistent because it is only driven by a bias. Compartment2
    # receives time varying input and thus the last value below threshold is
    # always different and thus it appears to be spiking at different levels.
    fig = plt.figure(1002)
    k = 1
    for j in range(0, 3):
        plt.subplot(3, 2, k)
        uProbes[j].plot()
        plt.title('u'+str(j))
        k += 1
        plt.subplot(3, 2, k)
        vProbes[j].plot()
        plt.title('v'+str(j))
        k += 1
    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_02_fig1002.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
