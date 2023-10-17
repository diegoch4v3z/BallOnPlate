# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2018-2021 Intel Corporation.
#
# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.
#
# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

# -----------------------------------------------------------------------------
# Tutorial: tutorial_18_learning.py
# -----------------------------------------------------------------------------
# This tutorial illustrates the use of the ESTDP learning rule. Two
# connections are connected via independent synapses to the same compartment
# 3 (c3). Connection conn13 (from compartment 1 (c1) to c3) is the driver
# for the compartment where "post-synaptic" spikes will be sent. Connection
# conn23 (from compartment 2 (c2) to c3) is the learning enabled connection
# where pre-synaptic spikes are sent and whose pre-synaptic traces (X) are
# inputs into the learning rule. Post synaptic traces (Y) from compartment 3
# back to the synapse of conn23 are configured as well. An illustration of
# the setup is shown below:
#                                                             _____________
# Post syn spikes  ---> conn13 synapse (no learning) ----->  |             |
#   from c1                                                  |compartment 3|
#                                                            |    (c3)     |
# Pre syn spikes   ---> conn23 synapse (with learning) ----> |_____________|
#   from c2        --->                                <----
#                pre-traces (X)                      post-traces (Y)

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

# For plotting without GUI
import matplotlib.pyplot as plt
import numpy as np
import nxsdk.api.n2a as nx
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API


# -----------------------------------------------------------------------------
# Function: setupNetwork
# This function sets up the network and returns the probes
# -----------------------------------------------------------------------------

def setupNetwork(net, spikeTimes0, spikeTimes1):
    # spikeTimes0: Spike generator spike times entering c1
    # spikeTimes1: Spike generator spike times entering c2

    # Compartment and Connection Prototypes

    # Threshold and synaptic weights are chosen so that c1 and c2 spike
    # immediately.
    # vThMant: Actual numerical threshold is 150*(2^6) = 9600.
    # compartmentCurrentDecay: current decay set to 1/1.25 (i.e.
    # 1/1.25*2^12 = 3276 where the 2^12 factor is for fixed point
    # implementation)
    # compartmentVoltageDecay: voltage decay set to 1/1.25 (i.e.
    # 1/1.25*2^12 = 3276 where the 2^12 factor is for fixed point
    # implementation)
    pbasic = nx.CompartmentPrototype(vThMant=150,
                                     compartmentCurrentDecay=3276,
                                     compartmentVoltageDecay=3276,
                                     logicalCoreId=0)
    # Threshold and synaptic weights are chosen so that "post-synaptic"
    # spikes entering from conn13 result in compartment 3 spikes, while the
    # "pre-synaptic" spikes entering from conn23 do not result in compartment
    # 3 spikes.
    # vThMant: Actual numerical threshold is 150*(2^6) = 9600.
    # compartmentCurrentDecay: current decay set to 1/1.25 (i.e.
    # 1/1.25*2^12 = 3276 where the 2^12 factor is for fixed point
    # implementation)
    # compartmentVoltageDecay: voltage decay set to 1/1.25 (i.e.
    # 1/1.25*2^12 = 3276 where the 2^12 factor is for fixed point
    # implementation)
    # enableSpikeBackprop: Enables back propagating action potentials
    # (post-traces)
    # enableSpikeBackpropFromSelf: Activate back propagating action potentials
    # when the compartment spikes
    plrn = nx.CompartmentPrototype(vThMant=150,
                                   compartmentCurrentDecay=3276,
                                   compartmentVoltageDecay=3276,
                                   enableSpikeBackprop=1,
                                   enableSpikeBackpropFromSelf=1,
                                   logicalCoreId=0)
    connProto1 = nx.ConnectionPrototype(weight=200, delay=0)
    # Configure ESTDP learning rule
    # dw: Weight adaptation learning rule (ESTDP).  In general, the learning
    # rule is a sum of products expression.
    # x1Impulse, x2Impulse: The increase in the trace value on the occurrence
    # of a pre-synaptic or post-synaptic spike.  Maximum value of 127.
    # x1TimeConstant, x2TimeConstant: Pre-synaptic and post-synaptic trace
    # exponential decay set to 1/4
    # tEpoch: The periodicity at which dynamic synaptic states are updated
    # for learning.  This parameter should in general be set as large as
    # possible (for faster execution) but small enough to ensure at most one
    # spike per tEpoch. A power of two will in general be most efficient.
    lr = net.createLearningRule(dw='2*x1*y0-2*y1*x0',
                                x1Impulse=40,
                                x1TimeConstant=4,
                                y1Impulse=40,
                                y1TimeConstant=4,
                                tEpoch=1)
    connProto2 = nx.ConnectionPrototype(weight=50, delay=0, enableLearning=1,
                                        learningRule=lr)

    # Create Compartments

    c1 = net.createCompartment(pbasic)
    c2 = net.createCompartment(pbasic)
    c3 = net.createCompartment(plrn)

    cg1 = net.createCompartmentGroup(size=0, prototype=pbasic)
    cg1.addCompartments(c1)
    cg1.addCompartments(c2)

    # Create Spike Generator

    numPorts = 2
    spikeGen = net.createSpikeGenProcess(numPorts)
    spikeGen.addSpikes([0, 1], [spikeTimes0, spikeTimes1])
    cMask = np.array([[1, 0],
                      [0, 1]])
    spikeGen.connect(cg1, prototype=connProto1, connectionMask=cMask)

    # Create Connections

    conn13 = net._createConnection(c1, c3, connProto1)
    conn23 = net._createConnection(c2, c3, connProto2)

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    conn13PrbSyn = conn13.probe([nx.ProbeParameter.SYNAPSE_WEIGHT])
    conn23PrbSyn = conn23.probe([nx.ProbeParameter.SYNAPSE_WEIGHT,
                                 nx.ProbeParameter.PRE_TRACE_X1])
    c3Probes = c3.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                         nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                         nx.ProbeParameter.SPIKE])
    probes = [conn13PrbSyn, conn23PrbSyn, c3Probes]

    return probes

# -----------------------------------------------------------------------------
# Function: createEstdpScatter
# This function processes the probed learning synapse weight along with
# injected pre-synaptic and post-synaptic spikes to create the data for a
# scatter plot of ESTDP behavior.
# -----------------------------------------------------------------------------


def createEstdpScatter(spikeTimes0ByC1, spikeTimes1ByC2, conn23PrbSyn):
    npPostTimes = np.array(spikeTimes0ByC1)
    npPreTimes = np.array(spikeTimes1ByC2)

    # Get data associated with adaptive weights
    weight = conn23PrbSyn[0].data
    time = conn23PrbSyn[0].timeVector

    npWgt = np.array(weight)
    npTime = np.array(time)

    difference = np.diff(npWgt)

    # "difference" is one element shorter than "npWgt".  Append a 0 at the
    # start to make "adjDiff" the same length as "npWgt".
    adjDiff = np.zeros(npWgt.shape)
    adjDiff[0] = 0
    adjDiff[1:] = difference

    # "wgtDeltas" contains where the weight changes by a non-zero amount
    # wgtDeltas: [-48, 10, -26, 24, -10, 50, 6, -68, 8, -32, 18, -12, 38, -4]
    wgtChangeIndices = np.where(adjDiff != 0)
    weightDeltas = adjDiff[wgtChangeIndices]

    # "deltaTimeStamps" contains the times at which the weights change
    # deltaTimeStamps: [14, 22, 27, 32, 40, 42, 52, 53, 62, 66, 72, 79, 82, 92]
    deltaTimeStamps = npTime[wgtChangeIndices]

    # "timeDelta" contains the pre minus post time associated with each
    # change in weight.
    # timeDelta: [-3, 7, -6, 4, -9, 1, 11, -2, 8, -5, 5, -8, 2, -1]
    timeDelta = []
    for learningInstance in range(0, weightDeltas.shape[0]):
        timeStamp = deltaTimeStamps[learningInstance]
        nearestPostTime = (npPostTimes[np.where(npPostTimes < timeStamp)])[-1]
        nearestPreTime = (npPreTimes[np.where(npPreTimes <= timeStamp)])[-1]
        timeDelta.append(nearestPostTime - nearestPreTime)

    return timeDelta, weightDeltas

# -----------------------------------------------------------------------------
# Function: createSpikeTimes
# This function sets up the relevant spike times for the tutorial.  It
# creates the times for the pre-synaptic and post-synaptic spikes
# -----------------------------------------------------------------------------


def createSpikeTimes():

    numSpikes = 10
    spikeInterval = [10, 13]

    # Post-synaptic spikes

    # spikeTimes0: [10, 20, 30, 40, 50, 60, 70, 80, 90]
    # Sent by the spike generator into compartment 1 (c1)
    spikeTimes0Base = (np.arange(1, numSpikes) * spikeInterval[0])
    spikeTimes0 = spikeTimes0Base.tolist()

    # spikeTimes0ByC1 = [11, 21, 31, 41, 51, 61, 71, 81, 91]
    # Spikes generated by compartment 1 (note the one time step delay as
    # compared to spikeTimes0)
    spikeTimes0ByC1 = (spikeTimes0Base + np.ones(spikeTimes0Base.shape,
                                                 dtype=int)).tolist()

    # Pre-synaptic spikes

    # spikeTimes1: [13, 26, 39, 52, 65, 78, 91, 104, 117]
    # Sent by the spike generator into compartment 2 (c2)
    spikeTimes1Base = (np.arange(1, numSpikes) * spikeInterval[1])
    spikeTimes1 = spikeTimes1Base.tolist()

    # spikeTimes1ByC2 = [14, 27, 40, 53, 66, 79, 92, 105, 118]
    # Spikes generated by compartment 2 (note the one time step dela yas
    # compared to spikeTimes1)
    spikeTimes1ByC2 = (spikeTimes1Base + np.ones(spikeTimes1Base.shape,
                                                 dtype=int)).tolist()

    return (spikeTimes0, spikeTimes1, spikeTimes0ByC1, spikeTimes1ByC2)


# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # -------------------------------------------------------------------------
    # Create a network
    # -------------------------------------------------------------------------

    net = nx.NxNet()

    # -------------------------------------------------------------------------
    # Create Spike Times
    # -------------------------------------------------------------------------

    (spikeTimes0, spikeTimes1, spikeTimes0ByC1, spikeTimes1ByC2) = \
        createSpikeTimes()

    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------

    probes = setupNetwork(net, spikeTimes0, spikeTimes1)

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    runTime = 100
    net.run(runTime)
    net.disconnect()

    # -------------------------------------------------------------------------
    # Unpack probes
    # -------------------------------------------------------------------------

    conn13PrbSyn = probes[0]
    conn23PrbSyn = probes[1]
    c3Probes = probes[2]

    # -------------------------------------------------------------------------
    # ESTDP Scatter Plot
    # -------------------------------------------------------------------------
    # The scatter plot illustrates the relationship between the time
    # difference between associated post synaptic spike and pre synaptic
    # spike ("time delta") and the change in weight ("weight change").
    # Notice that the Loihi ESTDP learning rule implementation exhibits the
    # typical ESTDP relationship.
    #
    # Note the outlier at time delta = -1 and weight change = -4.  The
    # reason for this is that there is both a pre-synaptic spike and a
    # post-synaptic spike occurring at that specific time step.  From
    # "wgtDeltas", "timeDelta", and "deltaTimeStamps" in createEstdpScatter,
    # it can be seen that this outlier occurs at time step 92.  There is a
    # pre-synaptic spike that occurs exactly at time step 92 and the post
    # synaptic spike at 91 results in a spike in the destination compartment
    # at time step 92. As a result, y0 and x0 are equal.  The value of y1
    # would be a decaying post-synaptic trace that was primarily from the
    # post synaptic spike at time step 81.  The value of x1 would be a decaying
    # pre-synaptic trace that was primarily from the pre synaptic spike at
    # time step 79.  Given that the impulse and delay for x1 and y1 are
    # identical, it is expected that x1 would have a smaller value. Thus,
    # given the form of the learning rule 2*x1*y0-2*y1*x0, it is expected
    # that dw would take a small negative value.

    (timeDelta, weightDeltas) = createEstdpScatter(spikeTimes0ByC1,
                                                   spikeTimes1ByC2,
                                                   conn23PrbSyn)

    fig1 = plt.figure(1000)
    plt.scatter(timeDelta, weightDeltas.tolist())
    plt.title('Weight Change vs. Time Delta')
    plt.xlabel('Time Delta (t_post - t_pre)')
    plt.ylabel('Weight Change')

    # -------------------------------------------------------------------------
    # Plot Compartment 3 Properties and Synapse Weights
    # -------------------------------------------------------------------------

    fig2 = plt.figure(1001)
    NUM_SUBPLOTS = 6
    plt.subplot(NUM_SUBPLOTS, 1, 1)
    c3Probes[0].plot()
    plt.title('c3 u0')
    plt.subplot(NUM_SUBPLOTS, 1, 2)
    c3Probes[1].plot()
    plt.title('c3 v0')
    plt.subplot(NUM_SUBPLOTS, 1, 3)
    c3Probes[2].plot()
    plt.title('c3 spikes 0')
    plt.subplot(NUM_SUBPLOTS, 1, 4)
    conn13PrbSyn[0].plot()
    plt.title('conn13 Wgt (Learning Disabled)')
    plt.subplot(NUM_SUBPLOTS, 1, 5)
    conn23PrbSyn[0].plot()
    plt.title('conn23 Wgt (Learning Enabled)')
    plt.subplot(NUM_SUBPLOTS, 1, 6)
    conn23PrbSyn[1].plot()
    plt.title('conn23 Pre-Trace (Learning Enabled)')

    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_18_fig1000.png"
        fileName2 = "tutorial_18_fig1001.png"
        print("No display available, saving to files " + fileName1 + " and " +
              fileName2)
        fig1.savefig(fileName1)
        fig2.savefig(fileName2)
