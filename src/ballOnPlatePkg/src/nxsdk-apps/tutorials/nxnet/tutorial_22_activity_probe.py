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
# Tutorial: tutorial_22_activity_probe.py
# -----------------------------------------------------------------------------
#
# This tutorial demonstrates how to set up activity probes to monitor spikes.
# The SDK offers spike probes which execute faster and are easier to set up
# but are limited by the available spike counters limited by the number of spike
# counters per chip. Per chip we offer around 2400 counters. As a workaround for the
# limited number of spike counters per chip, activity probes can be used instead.
# Since the update of the compartment activity variable is bound to the threshold
# homeostasis feature, threshold homeostasis must be enabled. In order to prevent
# the actual threshold to change the homeostasisGain parameter must be set to zero.
# In addition, since the activity variable is modeled as a spike trace, its decay
# time constant must be set to a large value for it to act as a spike counter.
# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

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


# Define a function to setup the network
def setupNetwork(net):
    # -----------------------------------------------------------------------
    # Configure core with 1 compartment driven by input spike
    # and activity probe setup to monitor spikes.
    # -----------------------------------------------------------------------
    # vThMant: Threshold voltage of the compartment.
    # enableHomeostasis: enableHomeostasis determines whether vThMant gets updated via activity
    #                    driven threshold homeostasis.
    # minActivity: minActivity is the lower threshold below which vThMant gets decreased to
    #              increase the compartment activity.
    # maxActivity: maxActivity is the upper threshold above which vThMant gets increased to
    #              reduce the compartment activity
    # homeostasisGain: homeostasisGain controls how strongly vThMant gets modified by activity
    #                  leaving the range defined by minActivity, maxActivity.
    # activityImpulse: activitiyImpulse specifies the amount by which activity increases for
    #                  each compartment spike.
    # activityTimeConstant: Decay time constant by which activity decays approximately exponentially
    #                       over time.
    # For activity counter to be used as spike counter, enableHomeostasis should be set to 1 and
    # homeostasisGain should be set to 0 as we are not really using homeostasis. For the activity
    # counter to accumulate spike values, the activityTimeConstant should be set to really high value.
    # ---------------------------------------------------------------------------

    cxPrototype = nx.CompartmentPrototype(
        vThMant=60,
        enableHomeostasis=1,
        minActivity=0,
        maxActivity=127,
        homeostasisGain=0,
        activityImpulse=1,
        activityTimeConstant=1000000,
    )

    cx1 = net.createCompartment(prototype=cxPrototype)

    # Create an activity probe and spike probe for the compartment and show the difference
    probes = []
    probes.append(cx1.probe([nx.ProbeParameter.SOMA_STATE_ACTIVITY]))
    probes.append(cx1.probe([nx.ProbeParameter.SPIKE],
                            probeConditions=nx.SpikeProbeCondition()))

    # Spike Generator
    numPorts = 1
    spikeGen = net.createSpikeGenProcess(numPorts)
    connProto1 = nx.ConnectionPrototype(weight=50)
    spikeGen.connect(cx1, prototype=connProto1)

    # List of spike times
    spikeTimes = [st for st in range(0, 100, 5)]

    spikeGen.addSpikes([0], [spikeTimes])

    # Return the configured probes
    return probes


if __name__ == "__main__":

    # Create a network
    net = nx.NxNet()

    # Setup the network
    probes = setupNetwork(net)

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    net.run(100)
    net.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------
    # The output will show how activity counter accumulates spikes as time goes on
    # and can be used as a spike counter.

    # Plotting the activity probe
    fig = plt.figure(1)
    plt.subplot(2, 1, 1)
    plt.title("Activity Probes")
    probes[0][0].plot()

    # Plotting the spike probe
    plt.subplot(2, 1, 2)
    plt.title("Spike Probes")
    probes[1][0].plot()
    plt.show()

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_22_fig1.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
