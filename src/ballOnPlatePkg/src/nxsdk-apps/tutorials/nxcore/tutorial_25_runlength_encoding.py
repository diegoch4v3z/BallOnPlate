"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

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

import os
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.nxinputgen.nxinputgen import BasicSpikeGenerator
import numpy as np
import matplotlib.pyplot as plt

def setupNetwork(numSteps, numSynPerAx):

    # ToDo: Why do I have to allocate more synapses than I need to avoid crash?
    board = N2Board(id=0, numChips=1, numCores=[1],
                    initNumSynapses=[[numSynPerAx*2]])

    core = board.n2Chips[0].n2CoresAsList[0]

    # Prepare 2 dummy compartments: One for absorbing all spikes from SPARSE
    # synapses and one for absorbing spikes from RUNLENGTH synapses
    core.cxProfileCfg[0].configure(decayU=4095, decayV=0)
    core.dendriteSharedCfg[0].configure(dsOffset=1)
    core.vthProfileCfg[0].staticCfg.configure(vth=2**17-1)
    core.numUpdates.configure(numUpdates=256)

    # Allocate axon for SPARSE synapses
    core.synapseMap[0].synapsePtr = 0
    core.synapseMap[0].synapseLen = numSynPerAx
    core.synapseMap[0].discreteMapEntry.configure()

    # Allocate axon for RUNLENGTH synapses
    core.synapseMap[1].synapsePtr = numSynPerAx
    core.synapseMap[1].synapseLen = numSynPerAx
    core.synapseMap[1].discreteMapEntry.configure()

    # Allocate two identical synapseFmts for SPARSE synapses
    core.synapseFmt[1].wgtBits = 7
    core.synapseFmt[1].numSynapses = 63
    core.synapseFmt[1].idxBits = 6
    core.synapseFmt[1].fanoutType = 1
    core.synapseFmt[1].compression = 0

    core.synapseFmt[2].wgtBits = 7
    core.synapseFmt[2].numSynapses = 63
    core.synapseFmt[2].idxBits = 6
    core.synapseFmt[2].fanoutType = 1
    core.synapseFmt[2].compression = 0

    # Allocate two identical synapseFmts for RUNLENGTH synapses
    core.synapseFmt[3].wgtBits = 7
    core.synapseFmt[3].numSynapses = 63
    core.synapseFmt[3].idxBits = 6
    core.synapseFmt[3].skipBits = 3
    core.synapseFmt[3].fanoutType = 1
    core.synapseFmt[3].compression = 1
    core.synapseFmt[3].wgtExp = 0

    core.synapseFmt[4].wgtBits = 7
    core.synapseFmt[4].numSynapses = 63
    core.synapseFmt[4].idxBits = 6
    core.synapseFmt[4].skipBits = 3
    core.synapseFmt[4].fanoutType = 1
    core.synapseFmt[4].compression = 1
    core.synapseFmt[4].wgtExp = 0

    # Allocate numSynPerAx synapses per axon but subdivided into
    # numSynPerEntry synapses per synaptic entry. Switching between synaptic
    # entries is enforced by switching to an identical synFmt at another
    # location.
    # All SPARSE synapses are connected to CIdx=0 while all RUNLENGTH
    # synapses are connected to CIdx=1. Both connections use the same (
    # random) weight. Regardless of encoding, but compartments must
    # accumulate the same synaptic activation.
    # both compartments must accumulate the same values
    j = 0
    sfinc = 0
    cxOffset = numSynPerAx
    for i in range(numSynPerAx):
        wgt = np.random.randint(-128, 128)
        core.synapses[i].configure(
            CIdx=j, synFmtId=1+sfinc, Wgt=wgt)
        core.synapses[i+numSynPerAx].configure(
            CIdx=j+cxOffset, synFmtId=3+sfinc, Wgt=wgt)
        j += 1
        if j == cxOffset // 2:
            sfinc = (sfinc + 1) % 2

    bs = BasicSpikeGenerator(board)
    for t in range(0, numSteps, 2):
        bs.addSpike(t, 0, 4, 0)
        bs.addSpike(t, 0, 4, 1)

    # Monitor voltage
    vProbesSparse = board.monitor.probe(core.cxState, range(cxOffset), 'v')
    vProbesRunlength = board.monitor.probe(core.cxState, range(cxOffset,128), 'v')

    # Debug
    if False:
        print("Synapses:")
        for i in range(numSynPerAx * 2):
            print(core.synapses[i])

        print("Synapse formats:")
        for i in range(4):
            print(core.synapseFmt[1+i])

    return board, vProbesSparse, vProbesRunlength


if __name__ == "__main__":

    numSteps = 10
    numSynPerAx = 700
    board, vProbesSparse, vProbesRunlength = setupNetwork(numSteps, numSynPerAx)

    # Run
    board.run(numSteps)
    board.disconnect()

    # Plot results
    plt.figure(1)

    plt.subplot(2, 1, 1)
    for p in vProbesSparse:
        p.plot()

    plt.subplot(2, 1, 2)
    for p in vProbesRunlength:
        p.plot()

    plt.show()
