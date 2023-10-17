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
# Tutorial: tutorial_20_soma_homeostasis.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces the homeostasis feature of the soma threshold and
# will demonstrate how it can be configured and how it works.
# Different examples show the possible solutions to change the membrane
# threshold of compartments.
#
# Description of the examples in the respective methods, which can be
# chosen in the main (comment in/out).
# Detailed description can be found above the plot sections.
#


# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

# For plotting without GUI
import numpy as np
import matplotlib.pyplot as plt
import nxsdk.api.n2a as nx
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API


def getCompartmentPrototype(vthMant, tau, tEpoch, spikeLevel, biasMant=0, biasExp=0, aMin=20, aMax=80, logicalCoreId=0):
    homeoStasisCxProto = nx.CompartmentPrototype(
        logicalCoreId=logicalCoreId,
        biasMant=biasMant,
        biasExp=biasExp,
        vThMant=vthMant,
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE,
        numDendriticAccumulators=64,
        compartmentCurrentDecay=410,
        compartmentVoltageDecay=int(1 / 16 * 2 ** 12),
        activityTimeConstant=tau,
        activityImpulse=spikeLevel,
        enableHomeostasis=1,
        minActivity=aMin,
        maxActivity=aMax,
        homeostasisGain=1,
        tEpoch=tEpoch)
    return homeoStasisCxProto


def getSpikeTimes():
    spikeTimes = np.append(np.arange(1, 300, 25), np.arange(301, 800, 10))
    spikeTimes = np.append(spikeTimes, np.arange(801, 1300, 20))
    spikeTimes = np.append(spikeTimes, np.arange(1301, 1800, 35))
    return spikeTimes.tolist()


def setupNetworkGeneralHomeostasisExample(net):
    compartmentPrototype = getCompartmentPrototype(
        vthMant=1000, tau=16, spikeLevel=40.0, tEpoch=1)
    homeostatisCompartment = net.createCompartment(
        prototype=compartmentPrototype)

    spikeGenerator = net.createSpikeGenProcess(numPorts=1)
    connproto = nx.ConnectionPrototype(
        weight=255, numWeightBits=-1, signMode=2)
    spikeGenerator.connect(homeostatisCompartment, prototype=connproto)
    spikeTimes = getSpikeTimes()
    spikeGenerator.addSpikes(0, spikeTimes)

    PP = nx.ProbeParameter
    probeParameters = [PP.COMPARTMENT_CURRENT, PP.SPIKE,
                       PP.SOMA_STATE_ACTIVITY, PP.SOMA_THRESHOLD_VOLTAGE]
    probes = homeostatisCompartment.probe(
        probeParameters, probeConditions=None)

    return probes


def setupNetworkAdaptiveThresholdHomeostasis(net):
    compartmentPrototype = getCompartmentPrototype(
        vthMant=500, biasMant=10,
        biasExp=6, tau=0, spikeLevel=100.0,
        tEpoch=4, aMin=2)
    adaptiveHomeostasisCx = net.createCompartment(
        prototype=compartmentPrototype)

    spikeGenerator = net.createSpikeGenProcess(numPorts=1)
    connproto = nx.ConnectionPrototype(
        weight=50, numWeightBits=-1, signMode=2, weightExponent=2)
    spikeGenerator.connect(adaptiveHomeostasisCx, prototype=connproto)
    spikeTimes = [2500]
    spikeGenerator.addSpikes(0, spikeTimes)

    PP = nx.ProbeParameter
    probeParameters = [PP.COMPARTMENT_CURRENT, PP.SPIKE,
                       PP.SOMA_STATE_ACTIVITY, PP.SOMA_THRESHOLD_VOLTAGE]
    probes = adaptiveHomeostasisCx.probe(probeParameters, probeConditions=None)

    return probes


def setupNetworkAdaptiveThresholdMulticompartmentNeuron(net):
    baseline_thr = 0.01
    scale = 10 ** 7
    beta = 1.8
    tau_a = 700
    tau_m = 20

    auxiliaryCompartmentPrototype = nx.CompartmentPrototype(
        logicalCoreId=0,
        vThMant=10000,
        compartmentCurrentDecay=int(2 ** 12),
        compartmentVoltageDecay=int(1 / tau_a * 2 ** 12)
    )

    mantissa = int((baseline_thr * scale) / (2 ** 6))
    mainCompartmentPrototype = nx.CompartmentPrototype(
        logicalCoreId=0,
        vThMant=1562,
        compartmentCurrentDecay=int(2 ** 12),
        compartmentVoltageDecay=int(1 / tau_m * 2 ** 12)
    )

    mainCompartmentPrototype.addDendrite(auxiliaryCompartmentPrototype,
                                         nx.COMPARTMENT_JOIN_OPERATION.ADD)

    np = nx.NeuronPrototype(mainCompartmentPrototype)
    n = net.createNeuron(np)

    weightAux = beta * (1 / (tau_a * tau_m) - 1 / (tau_a ** 2)) * scale
    wgt = int(weightAux / (2 ** 6))

    exp = 0
    for i in range(0, 8):
        if wgt % 2 == 0:
            exp += 1
            wgt = int(wgt / 2)
        else:
            break
    mainToAuxillaryConnectionProto = nx.ConnectionPrototype(weight=-wgt, weightExponent=exp,
                                                            signMode=nx.SYNAPSE_SIGN_MODE.INHIBITORY, numWeightBits=-1,
                                                            numDelayBits=1, compressionMode=nx.SYNAPSE_COMPRESSION_MODE.DENSE)

    n.connect(n.dendrites[0], prototype=mainToAuxillaryConnectionProto)

    spikeGenerator = net.createSpikeGenProcess(numPorts=2)
    inputWeight = 98
    inputWgtExp = 4
    spikeConnProto = nx.ConnectionPrototype(weight=inputWeight, weightExponent=inputWgtExp,
                                            signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY, numWeightBits=-1)
    spikeGenerator.connect(n.soma, prototype=spikeConnProto)
    spikeTimes = [100, 300, 600, 601]
    spikeGenerator.addSpikes(0, spikeTimes)

    PP = nx.ProbeParameter
    probeParameters = [PP.COMPARTMENT_CURRENT,
                       PP.COMPARTMENT_VOLTAGE, PP.SPIKE]
    mainProbes = n.soma.probe(probeParameters, probeConditions=None)
    auxProbes = n.dendrites[0].probe(probeParameters, probeConditions=None)
    probes = [auxProbes, mainProbes]

    # reference compartments
    referenceMainCompartment = net.createCompartment(mainCompartmentPrototype)
    referenceAuxillaryCompartmentPrototype = nx.CompartmentPrototype(
        logicalCoreId=0,
        vThMant=1562,
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE,
        compartmentCurrentDecay=int(2 ** 12),
        compartmentVoltageDecay=int(1/16 * 2 ** 12),
    )

    referenceAuxillaryCompartment = net.createCompartment(
        prototype=referenceAuxillaryCompartmentPrototype)
    spikeGenerator.connect(referenceAuxillaryCompartment,
                           prototype=spikeConnProto)

    refMainProbes = referenceMainCompartment.probe(
        probeParameters, probeConditions=None)
    refAuxProbes = referenceAuxillaryCompartment.probe(
        probeParameters, probeConditions=None)
    probes = probes + [refMainProbes, refAuxProbes]

    return probes


def generalHomeostasisExample():
    with nx.NxNet() as net:
        probes = setupNetworkGeneralHomeostasisExample(net)
        net.run(2000)

        uProbe = probes[0]
        spikeProbe = probes[1]
        aProbe = probes[2]
        vthProbe = probes[3]

        fig = plt.figure(1020)
        k = 1
        xposition = [300, 800, 1300, 1800]
        plt.subplot(2, 2, k)
        uProbe.plot()
        plt.title('u')
        # visually seperate the input rate changes
        for xc in xposition:
            plt.axvline(x=xc, color='k', linestyle='--')
        k += 1
        plt.subplot(2, 2, k)
        spikeProbe.plot()
        plt.title('spikes')
        # visually seperate the input rate changes
        for xc in xposition:
            plt.axvline(x=xc, color='k', linestyle='--')
        k += 1
        plt.subplot(2, 2, k)
        aProbe.plot()
        plt.title('a')
        # visually seperate the input rate changes
        for xc in xposition:
            plt.axvline(x=xc, color='k', linestyle='--')
        k += 1
        plt.axhline(y=20, color='tab:orange', linestyle='-.', label="aMin")
        plt.axhline(y=80, color='r', linestyle='-.', label="aMax")
        plt.legend()
        plt.subplot(2, 2, k)
        vthProbe.plot()
        plt.title('vth')
        # visually seperate the input rate changes
        for xc in xposition:
            plt.axvline(x=xc, color='k', linestyle='--')
        k += 1

        if haveDisplay:
            plt.show()
        else:
            fileName = "tutorial_20_fig_ex1.png"
            print("No display available, saving to file " + fileName + ".")
            fig.savefig(fileName)


def adaptiveThresholdHomeostasis():
    with nx.NxNet() as net:
        probes = setupNetworkAdaptiveThresholdHomeostasis(net)
        net.run(4500)

        uProbe = probes[0]
        spikeProbe = probes[1]
        aProbe = probes[2]
        vthProbe = probes[3]

        fig = plt.figure(1020)
        k = 1
        plt.subplot(2, 2, k)
        uProbe.plot()
        plt.title('u')

        k += 1
        plt.subplot(2, 2, k)
        spikeProbe.plot()
        plt.title('spikes')

        k += 1
        plt.subplot(2, 2, k)
        aProbe.plot()
        plt.title('a')

        k += 1
        plt.axhline(y=2, color='tab:orange', linestyle='-.', label="aMin")
        plt.axhline(y=80, color='r', linestyle='-.', label="aMax")
        plt.legend()
        plt.subplot(2, 2, k)
        vthProbe.plot()
        plt.title('vth')

        k += 1

        if haveDisplay:
            plt.show()
        else:
            fileName = "tutorial_20_fig_ex2.png"
            print("No display available, saving to file " + fileName + ".")
            fig.savefig(fileName)


def adaptiveThresholdMulticompartmentNeuron():
    with nx.NxNet() as net:
        probes = setupNetworkAdaptiveThresholdMulticompartmentNeuron(net)
        net.run(1000)
        fig = plt.figure(1020)
        k = 1
        legendList = [["decay compartment"], ["input compartment"], ["effective decay"],
                      ["input compartment without decay"]]
        for j, probe in enumerate(probes):
            uProbe = probe[0]
            vProbe = probe[1]
            spikeProbe = probe[2]
            # For each compartment plot the u, v and spike data
            plt.subplot(4, 3, k)
            uProbe.plot()
            plt.title('u')
            k += 1
            plt.subplot(4, 3, k)
            vProbe.plot()
            plt.title('v')
            plt.legend(legendList[j])
            k += 1
            plt.subplot(4, 3, k)
            spikeProbe.plot()
            plt.title('spikes')
            k += 1

        if haveDisplay:
            plt.show()
        else:
            fileName = "tutorial_20_fig_ex3.png"
            print("No display available, saving to file " + fileName + ".")
            fig.savefig(fileName)


# main
if __name__ == "__main__":
    generalHomeostasisExample()
    adaptiveThresholdHomeostasis()
    adaptiveThresholdMulticompartmentNeuron()
