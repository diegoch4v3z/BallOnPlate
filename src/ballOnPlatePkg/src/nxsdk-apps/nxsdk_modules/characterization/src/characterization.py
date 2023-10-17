# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
# 
# Copyright © 2020-2021 Intel Corporation.
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

"""Functions for implementing a test workload designed to isolate different neural computation operations"""

import numpy as np
import nxsdk.api.n2a as nx
from nxsdk.api.enums.api_enums import ProbeParameter
from nxsdk.graph.monitor.probes import PerformanceProbeCondition
import os
import inspect
from matplotlib import pyplot as plt
from nxsdk.arch.n2a.n2board import N2Board

def testBarrierSync(numChips=32,
                    bsCores=None,
                    boardName=None,
                    runTime=100000):
    """
    Runs barrier sync over a specified rectangular region of cores.
    
    :param numChips: (int) how many chips to replicate the workload over
    :param bsCores: [int, int] Logical coreid of the [first, last] core
    :param boardName: (str) Optionally specify a board to run on
    :param runTime: (int) Number of timesteps to run for
    
    :returns: A dictionary of power and timing components
    """     
    
    # Validate parameters
    if boardName is not None:
        os.environ['BOARD'] = boardName
    
    assert len(bsCores) == 2, 'bsCores format must be [firstCoreId, lastCoreId] if used'
    assert (bsCores[0]>=0) and (bsCores[1]>=0), 'bsCores must be positive'
    assert (bsCores[0]<128) and (bsCores[1]<128), 'bsCores must be less than 128'
    
    
    # Setup a dummy network with 2 neurons, one on each core requested by bsCores
    cxp = nx.CompartmentPrototype()
    net = nx.NxNet()
    srcCore = bsCores[0]
    sinkCore = bsCores[1]
    
    for chip in range(numChips):
        srcGrp = net.createCompartmentGroup(size=1,
                                            prototype=cxp,
                                            logicalCoreId=chip*128+srcCore)

        sinkGrp = net.createCompartmentGroup(size=1,
                                             prototype=cxp,
                                             logicalCoreId=chip*128+sinkCore)

    # compile the network
    compiler = nx.N2Compiler()
    board = compiler.compile(net)
    
    # remove all neurons
    for chip in board.n2Chips:
        for _, core in chip.n2Cores.items():
            core.numUpdates.configure(numUpdates=0)
    
    
    power = runBoard(board, runTime)
    return power
        
def testWorkload(numNeuronsPerCore=0, 
                 neuronState=None,
                 outputAxonsPerNeuron=0, 
                 spikeDistance=1, 
                 synapseMemWordsPerSpike=0, 
                 synapsesPerWord=0, 
                 bitsPerSynapse=1,
                 numChips=32, 
                 numTileRows=4,
                 coresPerTile=4,
                 runTime=100000, 
                 boardName=None):
    """
    Sets up a test workload with the provided parameters. If "bsCores" is passed a value other
    than "None", then a barrier sync test is run (no neurons updated) and the neuron parameters
    are ignored, otherwise a neurocore workload is executed.
    
    :param numNeuronsPerCore: (int) Number of neurons to implement per core
    :param neuronState: (str) Specifies whether neurons are "INACTIVE", "IDLE", or "SPIKING"
    :param outputAxonsPerNeuron: (int) Number of output axons (and therefore spikes) per neuron
    :param spikeDistance: (int) Distance each spike travels (in units of spike tiles)
    :param synapseMemWordsPerSpike: (int) Number of Synapse Memory words read per spike
    :param synapsesPerWord: (int) Number of Synapses in each Synapse Memory word
    :param bitsPerSynapse: (int) The size of each synapse in bits
    :param numChips: (int) how many chips to replicate the workload over
    :param numTileRows: (int) how many rows of tiles to use per chip
    :param numTileRows: (int) how many cores to use per sink or source tile
    :param runTime: (int) Number of timesteps to run for
    :param boardName: (str) Optionally specify a board to run on
    
    :returns: A dictionary of ops and a dictionary of power
    """
    
    # verify the parameters
    assert numNeuronsPerCore>0, "Except for barrier sync tests, all tests must have neurons"
    assert numNeuronsPerCore<=1024, "Too many neurons on a source core"
    assert outputAxonsPerNeuron<1024, "Too many output axons for a neuron"
    assert spikeDistance<=8, "Spikes can travel at most 8 router hops in this test"
    assert spikeDistance>=0, "Spikes must travel at least 0 hops"
    assert coresPerTile in [1,2,4], "Must have 1, 2, or 4 cores per router"
    assert synapseMemWordsPerSpike < 256, "At most 256 synapes mem words per spike"
    assert 0<coresPerTile<=4, "coresPerTile must be >0 and <=4"
    assert 0<numTileRows<=4, "numTileRows must be >0 and <=4"
    assert bitsPerSynapse*synapsesPerWord<=(64-4), "synapse words larger than 64 bits, reduce synapses per word or bits per synapse"

    if boardName is not None:
        os.environ['BOARD'] = boardName

    # specify parameters depending on the neuron mode requested
    if neuronState == "SPIKING":
        biasMant=1000
        vThMant=1
        decayV=1
        decayU=1
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE
    elif neuronState == "IDLE":
        biasMant=10
        vThMant=60000
        decayV=4000
        decayU=1
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.IDLE
    elif neuronState == "INACTIVE":
        biasMant=10
        vThMant=60000
        decayV=4000
        decayU=1
        functionalState=nx.COMPARTMENT_FUNCTIONAL_STATE.INACTIVE
    else:
        raise ValueError('Invalid neuronState, must be SPIKING, IDLE, or INACTIVE')
        
    # init the board
    numCoresPerChip = 128
    board = N2Board(1, numChips, [numCoresPerChip]*numChips, [[0]*numCoresPerChip]*numChips)
    
    # replicate across chips
    for chipNum in range(numChips):
        # replicate across mesh tile rows
        for tileRow in range(numTileRows):
            # replicate within tile
            for routerCoreNum in range(coresPerTile):
                setupSourceSinkPair(board,
                                    chipNum, 
                                    tileRow, 
                                    routerCoreNum,
                                    numNeuronsPerCore=numNeuronsPerCore,
                                    vThMant=vThMant,
                                    decayU=decayU,
                                    decayV=decayV,
                                    biasMant=biasMant,
                                    functionalState=functionalState,
                                    outputAxonsPerNeuron=outputAxonsPerNeuron,
                                    spikeDistance=spikeDistance,
                                    synapseMemWordsPerSpike=synapseMemWordsPerSpike,
                                    synapsesPerWord=synapsesPerWord,
                                    bitsPerSynapse=bitsPerSynapse)
                
    ops = calculateOps(numNeuronsPerCore=numNeuronsPerCore, 
                       neuronState=neuronState,
                       outputAxonsPerNeuron=outputAxonsPerNeuron, 
                       spikeDistance=spikeDistance, 
                       synapseMemWordsPerSpike=synapseMemWordsPerSpike, 
                       synapsesPerWord=synapsesPerWord, 
                       bitsPerSynapse=bitsPerSynapse,
                       numChips=numChips, 
                       numTileRows=numTileRows,
                       coresPerTile=coresPerTile)
    
    power = runBoard(board, runTime)
    
    return ops, power

def setupSourceSinkPair(board, 
                        chipNum,
                        tileRow, 
                        routerCoreNum,
                        numNeuronsPerCore=0,
                        vThMant=0,
                        decayU=0,
                        decayV=0,
                        biasMant=0,
                        functionalState=0,
                        outputAxonsPerNeuron=0,
                        spikeDistance=1,
                        synapseMemWordsPerSpike=0,
                        synapsesPerWord=0,
                        bitsPerSynapse=1):
    
    chip = board.n2Chips[chipNum]
    
    # identify the source and target cores
    coreNum = routerCoreNum + tileRow*4
    srcCore = chip.n2Cores[coreNum]

    # Configure the source core
    srcCore.vthProfileCfg[0].staticCfg.configure(vth=vThMant)
    srcCore.cxProfileCfg[0].configure(decayV=decayV,
                                      decayU=decayU)

    for ii in range((numNeuronsPerCore+3)//4):
        srcCore.cxMetaState[ii].configure(phase0=functionalState,
                                         phase1=functionalState,
                                         phase2=functionalState,
                                         phase3=functionalState)

    for ii in range(numNeuronsPerCore):
        srcCore.axonMap[ii].configure(ptr=0, 
                                      len=outputAxonsPerNeuron)

        srcCore.cxCfg[ii].configure(bias=biasMant,
                                    vthProfile=0,
                                    cxProfile=0)

    for ii in range(outputAxonsPerNeuron):
        srcCore.axonCfg[ii].discrete.configure(coreId=srcCore.id + (spikeDistance<<7),
                                               axonId=0)

    srcCore.dendriteAccumCfg.configure(delayBits=1)
    srcCore.numUpdates.configure(numUpdates=(numNeuronsPerCore+3)//4)

    # spikedistance==8 means send data off the edge of the mesh, so don't configure target core
    if spikeDistance<8:
        # configure the target core
        targetCore = chip.n2Cores[coreNum+16*spikeDistance] 
        targetCore.synapseFmt[1].configure(wgtBits=bitsPerSynapse,
                                           numSynapses=synapsesPerWord,
                                           idxBits=0,
                                           fanoutType=nx.SYNAPSE_SIGN_MODE.EXCITATORY,
                                           compression=nx.SYNAPSE_COMPRESSION_MODE.DENSE)

        targetCore.synapseMap[0].discreteMapEntry.configure(ptr=0, 
                                                            length=synapseMemWordsPerSpike)

        for ii in range(synapseMemWordsPerSpike):
            targetCore.synapseMem[ii].configure(dWord = 1 | (np.random.randint(1<<(synapsesPerWord*bitsPerSynapse))<<4))

        targetCore.dendriteAccumCfg.configure(delayBits=1)
        # spikedistance==0 means source and target cores are the same core, so don't set numNeurons to 0
        if spikeDistance>0:
            targetCore.numUpdates.configure(numUpdates=0)

    
    
def calculateOps(numNeuronsPerCore=0, 
                 neuronState=None,
                 outputAxonsPerNeuron=0, 
                 spikeDistance=1, 
                 synapseMemWordsPerSpike=0, 
                 synapsesPerWord=0, 
                 bitsPerSynapse=1,
                 numChips=32, 
                 numTileRows=4,
                 coresPerTile=4):    
    """
    Calculates the number of ops in the workload.
    
    :param numNeuronsPerCore: (int) Number of neurons to implement per core
    :param neuronState: (str) Specifies whether neurons are "INACTIVE", "IDLE", or "SPIKING"
    :param outputAxonsPerNeuron: (int) Number of output axons (and therefore spikes) per neuron
    :param spikeDistance: (int) Distance each spike travels (in units of spike tiles)
    :param synapseMemWordsPerSpike: (int) Number of Synapse Memory words read per spike
    :param synapsesPerWord: (int) Number of Synapses in each Synapse Memory word
    :param bitsPerSynapse: (int) The size of each synapse in bits
    :param numChips: (int) how many chips to replicate the workload over
    :param numTileRows: (int) how many rows of tiles to use per chip
    :param numTileRows: (int) how many cores to use per sink or source tile
    
    :returns: A dictionary of op counts
    """
    # calculate ops from simulation parameters
    ops = dict()
    ops['total'] = dict()
    ops['total']['neuronUpdates'] = numNeuronsPerCore*numTileRows*coresPerTile*numChips
    if neuronState=="SPIKING":
        ops['total']['spikes'] = ops['total']['neuronUpdates']*outputAxonsPerNeuron
        ops['total']['spikeHops'] = ops['total']['spikes']*spikeDistance
        ops['total']['synMemWords'] = ops['total']['spikes']*synapseMemWordsPerSpike
        ops['total']['synOps'] = ops['total']['synMemWords']*synapsesPerWord
    else:
        ops['total']['spikes'] = 0
        ops['total']['spikeHops'] = 0
        ops['total']['synMemWords'] = 0
        ops['total']['synOps'] = 0
    
    ops['perCore'] = dict()
    ops['perCore']['neuronUpdates'] = numNeuronsPerCore 
    if neuronState=="SPIKING":
        ops['perCore']['spikes'] = ops['perCore']['neuronUpdates']*outputAxonsPerNeuron
        ops['perCore']['synMemWords'] = ops['perCore']['spikes']*synapseMemWordsPerSpike
        ops['perCore']['synOps'] = ops['perCore']['synMemWords']*synapsesPerWord
    else:
        ops['perCore']['spikes'] = 0
        ops['perCore']['synMemWords'] = 0
        ops['perCore']['synOps'] = 0
    
    return ops

def runBoard(board, runTime):
    """
    Attaches power probes to a board and runs it for the specified number of timesteps.
    
    :param NxBoard board: The board to run
    :param int runTime: The number of timesteps to run
    
    :returns: The power profile dictionary from the board
    """
    # Attach energy probes
    bufferSize = 1024*2
    binSize = int(np.power(2, np.ceil(np.log2(runTime/bufferSize))))
    eProbe = board.probe(
         probeType=ProbeParameter.ENERGY,
         probeCondition=PerformanceProbeCondition(
             tStart=1,
             tEnd=runTime,
             bufferSize=bufferSize,
             binSize=binSize)
    )

    # run
    board.start()
    board.run(runTime)
    board.finishRun()
    board.disconnect()

    # extract power
    return board.energyTimeMonitor.powerProfileStats


def fitPowerPerOp(opCountPerTimestep, neuroCoreDynamicEnergyPerTimestep, opName, titleName=''):
    """
    Fits a line to measurements of Energy versus Operations to estimate the energy per op 
    and plots the result.
    
    :param opCountPerTimestep: list(int) Number of operations performed per timestep
    :param neuroCoreDynamicEnergyPerTimestep: list(int) Dynamic energy per timestep
    :param opName: (str) Name of the operation (for plot text)
    :param titleName: (str) Optional plot title (for plot text)
    """

    z = np.polyfit(opCountPerTimestep, neuroCoreDynamicEnergyPerTimestep, 1)
    xVal = np.array([0,opCountPerTimestep[-1]*1.1])
    plt.plot(xVal, z[1] + z[0]*xVal)
    plt.plot(opCountPerTimestep, neuroCoreDynamicEnergyPerTimestep, 'ro')
    plt.title('{} {:.1f} pJ/{}'.format(titleName, z[0]*1e6, opName))
    plt.xlabel('{} Per Timestep'.format(opName))
    plt.ylabel('Energy per Timestep (uJ)')
    plt.axis([xVal[0], xVal[1], 0, z[1] + z[0]*xVal[1]])
    plt.legend(['fit','measurements'])
    plt.show()
    
def fitTimePerOp(opCountPerCorePerTimestep, timePerTimestep, opName, titleName=''):
    """
    Fits a line to measurements of time versus operations to estimate the time per op 
    and plots the result.
    
    :param opCountPerCorePerTimestep: list(int) Number of operations performed per timestep
    :param timePerTimestep: list(int) Timestep duration
    :param opName: (str) Name of the operation (for plot text)
    :param titleName: (str) Optional plot title (for plot text)
    """
    z = np.polyfit(opCountPerCorePerTimestep, timePerTimestep, 1)
    xVal = np.array([0,opCountPerCorePerTimestep[-1]*1.1])
    plt.plot(xVal, z[1] + z[0]*xVal)
    plt.plot(opCountPerCorePerTimestep, timePerTimestep, 'ro')
    plt.title('{} {:.1f} ns/{}'.format(titleName, z[0]*1e3, opName))
    plt.xlabel('{} Per Core'.format(opName))
    plt.ylabel('Time Per Timestep (microseconds)')
    plt.axis([xVal[0], xVal[1], 0, z[1] + z[0]*xVal[1]])
    plt.legend(['fit','measurements'])
    plt.show()