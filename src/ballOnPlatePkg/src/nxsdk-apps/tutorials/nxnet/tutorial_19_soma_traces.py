# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2018-2021 Intel Corporation.

# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.

# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

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


def getCompartmentPrototype(logicalCoreId, tau, tEpoch):
    # create compartment prototype with homeostasis parameters enabled
    homeoStasisCxProto = nx.CompartmentPrototype(
        logicalCoreId=logicalCoreId,
        vThMant=4,
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE,
        numDendriticAccumulators=64,
        compartmentCurrentDecay=410,
        compartmentVoltageDecay=256,
        activityTimeConstant=tau,
        activityImpulse=40.0,
        enableHomeostasis=1,
        minActivity=0,
        maxActivity=127,
        homeostasisGain=0,
        tEpoch=tEpoch)
    return homeoStasisCxProto


def setupNetwork(net):
    tauList = [8, 8, 8, 16, 16, 16]
    tEpochList = [1, 8, 16, 1, 8, 16]
    spikeTimes = [5, 25, 45, 65]
    numCompartments = len(tauList)

    compartmentPrototypes = []
    for logicalCoreId, (tau, tEpoch) in enumerate(zip(tauList, tEpochList)):
        compartmentPrototype = getCompartmentPrototype(
            logicalCoreId, tau, tEpoch)
        compartmentPrototypes.append(compartmentPrototype)

    compartmentGroup = net.createCompartmentGroup(size=numCompartments, prototype=compartmentPrototypes,
                                                  prototypeMap=range(numCompartments))

    spikeGenerator = net.createSpikeGenProcess(numPorts=1)
    connproto = nx.ConnectionPrototype(weight=1, numWeightBits=-1, signMode=2)
    spikeGenerator.connect(compartmentGroup, prototype=connproto)
    spikeGenerator.addSpikes(0, spikeTimes)

    probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT, nx.ProbeParameter.SPIKE,
                       nx.ProbeParameter.SOMA_STATE_ACTIVITY]
    probes = compartmentGroup.probe(probeParameters, probeConditions=None)

    return probes, tEpochList


def genPlots(probes, tEpochList):
    fig = plt.figure(10019)
    k = 1
    numCompartments = len(probes)
    # For each compartment plot the u, v, spike and a variables

    for j in range(0, numCompartments):
        uProbe = probes[j][0]
        spikeProbe = probes[j][1]
        somaProbe = probes[j][2]
        plt.subplot(numCompartments, 3, k)
        uProbe.plot()
        plt.title('u' + str(j))
        k += 1
        plt.subplot(numCompartments, 3, k)
        spikeProbe.plot()
        plt.title('spikes' + str(j))
        k += 1
        plt.subplot(numCompartments, 3, k)
        somaProbe.plot()
        plt.title('a' + str(j))
        plt.legend(["tepoch = " + str(tEpochList[j])])
        k += 1

    fig.text(0.94, 0.5, '<-- tau=16        tau=8  -->',
             ha='center', va='center', rotation='vertical')

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_19_fig1019.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)


def run_tutorial19_soma_traces():
    net = nx.NxNet()
    probes, tEpochList = setupNetwork(net)
    net.run(100)
    net.disconnect()
    genPlots(probes, tEpochList)


# main
if __name__ == "__main__":
    run_tutorial19_soma_traces()
