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
# This function sets up the network and returns the board
# -----------------------------------------------------------------------------


def setupNetwork(net):

    # Create a compartment prototype with the following parameters:
    # biasMant: Configure bias mantissa.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 100*(2^6) = 6400..
    # biasExp: Configure bias exponent.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 100*(2^6) = 6400.
    # vThMant: Configuring voltage threshold value mantissa. Actual numerical
    #   threshold is vThMant*(2^6) = 1000*2^6 = 64000
    # functionalState: The setting to PHASE_IDLE = 2 allows the compartment to
    #   be driven by a bias current alone.  See user guide for all possible
    #   phases.
    # compartmentVoltageDecay: Set to 1/16 (2^12 factor for fixed point
    #   implementation)
    p = nx.CompartmentPrototype(biasMant=100,
                                biasExp=6,
                                vThMant=1000,
                                functionalState=2,
                                compartmentVoltageDecay=256)

    # Create a compartment in the network using the prototype
    compartment = net.createCompartment(p)
    # Create a compartment probe to probe the compartment states: compartment current(U) and compartment voltage(V)
    # probeConditions=None implies default probe conditions will be used for each probe
    probes = compartment.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                                nx.ProbeParameter.COMPARTMENT_VOLTAGE], probeConditions=None)

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

    fig = plt.figure(1001)
    # Since there are no incoming spikes and noise is disabled by default, u
    # remains constant at 0
    plt.subplot(1, 2, 1)
    uProbe = probes[0]
    uProbe.plot()
    plt.title('u')

    # v increases due to the bias current.  The concave nature of the curve
    # when the voltage is increasing is due to the decay constant.  Upon
    # reaching the threshold of 64000, the compartment spikes and resets to 0.
    # Since there is no refractory period, the voltage immediate begins to
    # increase again.
    plt.subplot(1, 2, 2)
    vProbe = probes[1]
    vProbe.plot()
    plt.title('v')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_01_fig1001.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
