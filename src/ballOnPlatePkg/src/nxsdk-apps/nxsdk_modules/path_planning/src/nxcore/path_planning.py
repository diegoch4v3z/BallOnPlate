# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2019-2021 Intel Corporation.
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
# expressly stated in the License

"""
Path planning module is based on the planning algorithm modeled after
the operational principles of hippocampal place cells. The algorithm
infers associations between neurons in a network from the asymmetric
effects of STDP on a propagating sequence of spikes.
"""

# -----------------------------------------------------------------------------
# Import modules
# -----------------------------------------------------------------------------

import os
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.monitor.probes import SpikeProbeCondition
from nxsdk.graph.processes.phase_enums import Phase
from collections import defaultdict, deque
from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from struct import *
import glob
import nxsdk.compiler.microcodegen.interface as uci
from nxsdk.logutils.nxlogging import get_logger

class PathPlanning:
    """
    Module : Given a graph, a source and destination finds the shortest path
    """

    def __init__(self, graph):
        """
        Initializes the module
        :param graph: Directory of the graph
        """
        self.graph = graph
        # Create logger for the module
        self.logger = get_logger("NET.PATH_PLANNING")
        self.shortestPath = []

    def setupNetwork(self):
        """
        Creates the board with required resource for the graph.
        :return: Dictionary of ChipInfo and Board
        """
        # Get the Chip Binaries from the graph
        chipBinaries = glob.glob(os.path.join(self.graph, 'chip*.binary'))
        numChips = len(chipBinaries)

        self.logger.info("Number of chips : {}".format(numChips))

        # Initialize the board
        # N2board ID
        boardId = 1
        # Number of cores per chip
        numCoresPerChip = []
        numSynapsesPerCore = []
        for i in range(numChips):
            numCoresPerChip.append(128)
            # Number of synapses per core
            numSynapses = []
            for j in range(numCoresPerChip[i]):
                numSynapses.append(100000)
            numSynapsesPerCore.append(numSynapses)

        self.board = N2Board(
            boardId,
            numChips,
            numCoresPerChip,
            numSynapsesPerCore)
        chipInfo = {}
        self.board.options = ['epoch=8']

        # Configure network
        for i in range(numChips):
            chipId = int(chipBinaries[i][chipBinaries[i].rfind(
                'chip') + 4:chipBinaries[i].rfind('.binary')])
            self.logger.info(
                "Reading File : {} chipId : {}".format(
                    chipBinaries[i], chipId))
            chipInfo[chipId] = self.createConnectionsFromFile(
                chipBinaries[i], chipId)
            self.logger.info("numCores: {}".format(
                len(chipInfo[chipId]['coreMap'])))

        # Setup all channels
        self.spikeTimeChannels = [self._setupSnip(i) for i in range(numChips)]

        self.firstRun = True

        # Return the configured board
        return chipInfo, self.board

    def _setNetworkState(self, chipInfo):
        """
        Change learning rule to set weight back to initial state.
        """
        for chipId in range(len(chipInfo)):
            for coreId in range(len(chipInfo[chipId]['coreMap'])):
                currentCore = self.board.n2Chips[chipId].n2Cores[coreId]
                currentCore.synapseFmt[1].stdpProfile = 1
                currentCore.stdpPreProfileCfg[0].configure(updateAlways=1)

    def _resetNetworkState(self, chipInfo):
        """
        Change learning rule to graph search.
        """
        for chipId in range(len(chipInfo)):
            for coreId in range(len(chipInfo[chipId]['coreMap'])):
                currentCore = self.board.n2Chips[chipId].n2Cores[coreId]
                currentCore.synapseFmt[1].stdpProfile = 0
                currentCore.stdpPreProfileCfg[0].configure(updateAlways=0,
                                                           updateOnX1=1)

    def configureCore(self, core):
        """
        Given the core, configures various nodesets
        :param core: Core to be configure
        """
        core.cxProfileCfg[0].configure(decayV=3000,
                                       decayU=4095,
                                       bapAction=3,
                                       refractDelay=63)

        core.dendriteSharedCfg.configure(posVmLimit=7,
                                         negVmLimit=0,
                                         dsOffset=0)

        # Configure membrane potential threshold value mantissa.
        # Actual numerical threshold is vth*(2^6) = 1*2^6 = 64
        core.vthProfileCfg[0].staticCfg.configure(vth=1)

        core.dendriteAccumCfg.delayBits = 3

        # Initialize dendriteTimeState
        core.dendriteTimeState.configure(tepoch=8)

        # Configure synapseFmt
        core.synapseFmt[1].stdpProfile = 0
        core.synapseFmt[1].learningCfg = 3
        core.synapseFmt[1].dlyBits = 3
        core.synapseFmt[1].wgtBits = 7
        core.synapseFmt[1].numSynapses = 63
        core.synapseFmt[1].idxBits = 5
        core.synapseFmt[1].fanoutType = 2

        # --------------------------------------------------------------------------
        # Configure Learning parameters
        # Configure at what synMapId the first learning enabled axon starts
        core.stdpCfg[0].firstLearningIndex = 0

        # TODO: Change numTraceHist to 1 for synapse delay > 1
        core.stdpPreProfileCfg[0].configure(updateOnX1=1,
                                            numTraceHist=0)

        # Configure learning rule
        antiHebbianlearningRule = 'dw = x0*y1 - y0*x1'
        #antiHebbianlearningRuleOnlyDepress = 'dw = - y0*x1'
        antiHebbianlearningRuleForEnergy = 'dw = 0*x0*y1 - 0*y0*x1'
        learningRuleForResettingWeights = 'dw= -w*u0 + u0*2'

        # The ruleToCode utility generates the required uCode.
        uc = uci.ruleToUCode([antiHebbianlearningRule], False)
        for ct in range(0, uc.numUCodes):
            core.stdpUcodeMem[ct].word = uc.uCodes[ct]
        # uCodePtr points to the learning rule in the stdpUCodeMem.
        core.stdpProfileCfg[0].configure(uCodePtr=0,
                                         requireY=0,
                                         decimateExp=uc.decimateExponent,
                                         numProducts=uc.numProducts)

        # The ruleToCode utility generates the required uCode.
        uc = uci.ruleToUCode([learningRuleForResettingWeights], False)
        for ct in range(0, uc.numUCodes):
            core.stdpUcodeMem[ct+1].word = uc.uCodes[ct]
        # uCodePtr points to the learning rule in the stdpUCodeMem.
        core.stdpProfileCfg[1].configure(uCodePtr=1,
                                         requireY=0,
                                         decimateExp=uc.decimateExponent,
                                         numProducts=uc.numProducts)

        # Configure pre trace behavior (trace decay time constant and the level
        # by which a trace gets incremented upon each spike). A helper class
        # (TraceCfgGen) is available to generate the register configuration based
        # on the time constant (tau) and increment (spikeLevel (integer and
        # fractional parts)) and writes it to the appropriate registers.
        # decay = [116,105,86,58,26,5]
        # tau = tcg.calcTau(decay)
        tcg = TraceCfgGen()
        tc = tcg.genTraceCfg(tau=10,
                             spikeLevelInt=127,
                             spikeLevelFrac=100)
        tc.writeToRegister(core.stdpPreCfg[0])

    def configureCompartment(self, core, compartmentId):
        """
        Configures Cx with compartmenId for given Core
        :param core: Core to which Cx belongs
        :param compartmentId: Id of the Cx
        """

        core.cxCfg[compartmentId].configure(bias=1,
                                            biasExp=0,
                                            vthProfile=0,
                                            cxProfile=0)
        core.cxMetaState[int(compartmentId / 4)].configure(phase0=2,
                                                           phase1=2,
                                                           phase2=2,
                                                           phase3=2)

    def configureNumUpdates(self, core, numCompartments, numSynapses):
        """
        Configure number of active Cxs and number of learning synapses.
        :param core: Core to be configured
        :param numCompartments: Number of configured Cx for the Core
        :param numSynapses: Number of Learning Synapses
        :return:
        """
        core.numUpdates.configure(numUpdates=int((numCompartments + 3) / 4),
                                  numStdp=numSynapses)

    def configureSynapse(
            self,
            core,
            synMapOffset,
            synMapEntry,
            synBase,
    ):
        """
        Configures the synaptic connection
        """
        for i in range(len(synMapEntry['compartmentId'])):
            core.synapses[i + synBase].CIdx = synMapEntry['compartmentId'][i]
            core.synapses[i + synBase].Wgt = synMapEntry['weight'][i]
            assert core.synapses[i + synBase].Wgt == 2
            core.synapses[i + synBase].Dly = synMapEntry['delay'][i]
            core.synapses[i + synBase].synFmtId = 1
            core.synapses[i + synBase].LrnEn = 1
            self.logger.debug(
                "Debug: synBase: {}"
                " i: {}"
                " CIdx: {}"
                " Wgt: {}"
                "Dly: {}".format(
                    synBase,
                    i,
                    synMapEntry['compartmentId'][i],
                    synMapEntry['weight'][i],
                    synMapEntry['delay'][i]))

        # Configure synapseMap entry
        core.synapseMap[synMapOffset].synapsePtr = synBase
        core.synapseMap[synMapOffset].synapseLen = len(
            synMapEntry['compartmentId'])
        core.synapseMap[synMapOffset].discreteMapEntry.configure()
        core.synapseMap[synMapOffset +
                        1].singleTraceEntry.configure(preProfile=0, tcs=0)

    def configureAxon(
            self,
            core,
            compartmentId,
            targetChipId,
            targetcoreId,
            targetAxonId):
        """
        Configures the output Axon.
        :param core: Core to be configured
        :param compartmentId: Id of the Cx
        :param targetChipId: ChipId of the destination input axon
        :param targetcoreId: CoreId of the destination input axon
        :param targetAxonId: AxonId of the destination input axon
        """
        core.createDiscreteAxon(
            compartmentId,
            targetChipId,
            targetcoreId,
            targetAxonId)

    def getLogicalCoreNum(self, coreNum):
        """
        Utility function to return logicalCoreId given coreNum
        :param coreNum: Input coreNum
        :return: LogicalCoreId
        """
        x = coreNum >> 7
        y = (coreNum >> 2 & 31) - 1
        p = coreNum & 3
        logicalCoreNum = x << 4 | y << 2 | p
        return logicalCoreNum

    def getLogicalChipId(self, chipId):
        """
        Utility function to return LogicalChipId, given physical ChipId
        :param chipId: Physical ChipId
        :return: LogicalChipId
        """
        chipIdTable = list(
            self.board.executor.logistics.boardConfiguration.chipIds)
        logicalChipId = chipIdTable.index(chipId)
        return logicalChipId

    def createConnectionsFromFile(self, fileName, mychipId):
        """
        Given filename and chipId, create connections by reading from file.
        """
        chipInfo = defaultdict(list)
        chipConnections = open(fileName, "rb")
        numCores = int(unpack('i', chipConnections.read(4))[0])
        self.logger.debug("numCores: {}".format(numCores))
        for i in range(numCores):
            coreNum = int(
                self.getLogicalCoreNum(
                    unpack(
                        'i',
                        chipConnections.read(4))[0]))
            numCompartments = int(unpack('i', chipConnections.read(4))[0])
            chipInfo['coreMap'].append(coreNum)
            chipInfo['compartmentMap'].append(numCompartments)
            core = self.board.n2Chips[mychipId].n2Cores[coreNum]
            self.configureCore(core)
            self.logger.debug(
                "coreNum: {} numCompartments: {}".format(
                    coreNum, numCompartments))

            for j in range(numCompartments):
                compartmentId = int(unpack('i', chipConnections.read(4))[0])
                self.configureCompartment(core, compartmentId)
                numFanoutCores = int(unpack('i', chipConnections.read(4))[0])
                self.logger.debug(
                    "\tcompartmentId: {} numFanoutCores: {}".format(
                        compartmentId, numFanoutCores))
                for k in range(numFanoutCores):
                    fanoutChipId = int(unpack('i', chipConnections.read(4))[0])
                    # Dont need logical coreID
                    fanoutCoreId = int(unpack('i', chipConnections.read(4))[0])
                    fanoutSynMapOffset = int(
                        unpack('i', chipConnections.read(4))[0])
                    self.configureAxon(
                        core,
                        compartmentId,
                        fanoutChipId,
                        fanoutCoreId,
                        fanoutSynMapOffset)
                self.configureAxon(
                    core,
                    compartmentId,
                    mychipId,
                    #core.parent.xypToCoreId(9, 1, 0),
                    core.parent.xypToCoreId(5, 0, 0),
                    0x20)
            numSynMapEntries = int(unpack('i', chipConnections.read(4))[0])
            chipInfo['synapseMap'].append(numSynMapEntries)
            self.configureNumUpdates(core, numCompartments, numSynMapEntries)
            self.logger.debug("numSynMapEntries: {}".format(numSynMapEntries))
            synBase = 0
            for j in range(numSynMapEntries):
                debug = 0
                synMapOffset = int(unpack('i', chipConnections.read(4))[0])
                numSynCompartments = int(
                    unpack('i', chipConnections.read(4))[0])
                synMapEntry = defaultdict(list)
                for k in range(numSynCompartments):
                    synMapEntry['compartmentId'].append(
                        int(unpack('i', chipConnections.read(4))[0]))
                    synMapEntry['weight'].append(
                        int(unpack('i', chipConnections.read(4))[0]))
                    synMapEntry['delay'].append(
                        int(unpack('i', chipConnections.read(4))[0]))

                self.configureSynapse(
                    core, synMapOffset, synMapEntry, synBase)
                synBase += len(synMapEntry['compartmentId'])
                if synBase >= 100000:
                    raise Exception("Total number of synapses exceed 100000")
        chipConnections.close()
        return chipInfo

    def readTargets(self, graphDir, targetNum, physicalCoreId=True):
        """
        Read Target From the file
        :param graphDir: Directory of the graph
        :param targetNum: Target Id
        """
        targetList = []
        fileName = 'targetLocation' + str(targetNum) + '.binary'
        targetBinary = os.path.join(graphDir, fileName)
        targetFile = open(targetBinary, "rb")
        numTargets = int(unpack('i', targetFile.read(4))[0])
        for i in range(numTargets):
            target = {}
            target['chipId'] = int(unpack('i', targetFile.read(4))[0])
            if(physicalCoreId):
                target['coreId'] = int(
                    self.getLogicalCoreNum(
                        unpack(
                            'i',
                            targetFile.read(4))[0]))
            else:
                target['coreId'] = int(unpack('i', targetFile.read(4))[0])
            target['compartmentId'] = int(unpack('i', targetFile.read(4))[0])
            self.logger.info(
                "Target i: {}"
                " chipId: {}"
                " coreId: {}"
                " compartmentId: {}".format(
                    i,
                    target['chipId'],
                    target['coreId'],
                    target['compartmentId']))
            targetList.append(target)
        targetFile.close()
        return targetList

    def setTargetBias(self, target, bias):
        """
        Set the bias for the target
        """
        core = self.board.n2Chips[target['chipId']].n2Cores[target['coreId']]
        self.logger.info(
            "Setting target Bias: {}"
            " chipId: {}"
            " coreId: {}"
            " compartmentId: {}".format(
                bias,
                target['chipId'],
                target['coreId'],
                target['compartmentId']))

        core.cxCfg[target['compartmentId']].configure(bias=bias,
                                                      biasExp=0,
                                                      vthProfile=0,
                                                      cxProfile=0)

    def search(self, chipInfo, source, target, maxLength=2**31-1):
        """
        Run search to find shortest path between source and target at most
        maxLength long.
        :return: True if path found, False otherwise.
        """
        if self.firstRun:
            self.firstRun = False
            self.board.start()

            # Disable all spikes to LMT
            for chipId in range(len(chipInfo)):
                for coreId in range(len(chipInfo[chipId]['coreMap'])):
                    currentCore = self.board.n2Chips[chipId].n2Cores[coreId]
                    for entry in currentCore.axonMap:
                        if entry.len > 0:
                            entry.len -= 1
                            assert currentCore.axonCfg[entry.ptr + entry.len].discrete.coreId == \
                                   currentCore.parent.xypToCoreId(5, 0, 0)
        else:
            self._setNetworkState(chipInfo)
            self.board.run(8)
            self._resetNetworkState(chipInfo)

        # Enable spike to LMT for source
        sourceCore = self.board.n2Chips[source['chipId']].n2Cores[source['coreId']]
        sourceAxonMap = sourceCore.axonMap[source['compartmentId']]
        sourceAxonMap.len += 1

        self.setTargetBias(target, (1 << 6) + 1)

        self.board.run(1)

        # Reset target bias to prevent spiking
        self.setTargetBias(target, 1)

        self.board.run(maxLength, aSync=True)
        spikeTimeChannel = None
        while not spikeTimeChannel:
            runComplete = self.board.isRunComplete()
            for channel in self.spikeTimeChannels:
                if channel.probe() > 0:
                    spikeTimeChannel = channel
                    break
            if runComplete:
                break
        if spikeTimeChannel:
            spikeTime = spikeTimeChannel.read(2)
            self.board.pause()
            self.board.finishRun()

        # Disable spike to LMT for source
        sourceAxonMap.len -= 1

        return spikeTimeChannel is not None

    def searchThenTrace(self, chipInfo, source, target, maxLength=2**31-1):
        """
        Run search to find shortest path between source and target at most
        maxLength long, and return the shortest path or None.
        :return: Shortest path or None
        """
        found = self.search(chipInfo, source, target, maxLength=maxLength)
        return self.tracePathThroughSynapseWeights(source, target) if found else None

    def _setupSnip(self, snipChip):
        """
        Creates snip to check if Source Cx spikes and sends the spike timing of the
        on the channel.
        :param snipChip: ChipId of Source Cx
        """
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/runspike.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "run_spike"
        guardName = "do_run_spike"
        runSpikeProcess = self.board.createSnip(
            phase=Phase.EMBEDDED_SPIKING,
            name="runSpike",
            cFilePath=cPath,
            includeDir=includeDir,
            funcName=funcName,
            guardName=guardName,
            chipId=snipChip)

        # Create a channel named nxspktime for getting the value of spike
        # timing
        spikeTimeChannel = self.board.createChannel("nxspktime_{}".format(snipChip),
                                                    numElements=100,
                                                    messageSize=4)
        spikeTimeChannel.connect(runSpikeProcess, None)
        return spikeTimeChannel

    def setupSnip(self, snipChip):
        """
        Returns the channel associated with the given chip.
        :param snipChip: ChipId of chip
        """
        return self.spikeTimeChannels[snipChip]

    def getNextEntry(self, currentAxonList):
        """
        Traverses the axonList and finds the next node in the path
        :param currentAxonList: List of fanning out connecttions
        """
        nextEntry = {}
        for i in range(len(currentAxonList)):
            nextEntry['chipId'] = self.getLogicalChipId(
                currentAxonList[i][1]) if (
                currentAxonList[i][0] == 1) else currentAxonList[i][1]
            nextEntry['coreId'] = self.getLogicalCoreNum(currentAxonList[i][2])
            # Ignore LMT which comes with a negative logicalCoreId
            if (nextEntry['coreId'] < 0):
                continue
            nextSynapseMapOffset = currentAxonList[i][3]
            nextCore = self.board.n2Chips[nextEntry['chipId']
                                          ].n2Cores[nextEntry['coreId']]
            nextCoreSynapses = self.board.executor.logistics.synapseCompiler.decodeSynapsesMapEntry(
                nextCore, nextSynapseMapOffset)
            nextSynapseMapEntry = nextCore.synapseMap[nextSynapseMapOffset].discreteMapEntry
            for j in range(nextSynapseMapEntry.synapseLen):
                nextSynapse = nextCoreSynapses[j]
                if(nextSynapse.Wgt > 0):
                    nextEntry['compartmentId'] = nextSynapse.CIdx
                    return nextEntry
        self.logger.error("Error: Trace function failed to find next entry")
        return None

    def tracePathThroughSynapseWeights(self, source, target):
        """
        Traces the shortest path
        :param source: Source Cx
        :param target: Target Cx
        :return: Returns the shortest path
        """
        currentEntry = source
        self.shortestPath.clear()
        self.shortestPath.append(source)
        i = 0
        self.logger.info("List of nodes in the shortest path are:")
        while(bool(currentEntry) and currentEntry != target):
            currentChipId = currentEntry['chipId']
            currentCoreId = currentEntry['coreId']
            currentCompartmentId = currentEntry['compartmentId']
            self.logger.info(
                "Node: {}"
                " Chip: {}"
                " Core: {}"
                " Compartment: {}".format(
                    i,
                    currentChipId,
                    currentCoreId,
                    currentCompartmentId))

            currentCore = self.board.n2Chips[currentChipId].n2Cores[currentCoreId]
            currentAxonList = currentCore.axonMap[currentCompartmentId].decodeDiscreteAxons(
            )
            currentEntry = self.getNextEntry(currentAxonList)
            if currentEntry is None:
                return None
            self.shortestPath.append(currentEntry)
            i = i + 1

        self.logger.info(
            "Node: {} "
            " Chip: {} "
            " Core: {} "
            " Compartment: {}".format(
                i,
                currentEntry['chipId'],
                currentEntry['coreId'],
                currentEntry['compartmentId']))
        return self.shortestPath

    def readAllAxonsAndSynapses(self, chipInfo):
        """
        Debug functions which reads all axons and synapses
        :param chipInfo: Dictionary of ChipInfo
        """
        for chipId in range(len(chipInfo)):
            for coreId in range(len(chipInfo[chipId]['coreMap'])):
                currentCore = self.board.n2Chips[chipId].n2Cores[coreId]
                for i in range(chipInfo[chipId]['compartmentMap'][coreId]):
                    currentAxonList = currentCore.axonMap[i].decodeDiscreteAxons(
                    )
                    for j in range(len(currentAxonList)):
                        nextChipId = self.getLogicalChipId(currentAxonList[j][1]) if (
                            currentAxonList[j][0] == 1) else currentAxonList[j][1]
                        nextCoreId = self.getLogicalCoreNum(
                            currentAxonList[j][2])
                        nextSynapseMapOffset = currentAxonList[j][3]
                        self.logger.info(
                            "Core: {}"
                            " Compartment: {}"
                            " Fanout chipId: {}"
                            " coreId: {}"
                            " synMapOffset: {}".format(
                                coreId,
                                i,
                                nextChipId,
                                nextCoreId,
                                nextSynapseMapOffset))
                self.logger.info("-------------------------------------")

                for k in range(chipInfo[chipId]['synapseMap'][coreId]):
                    currentCoreSynapses = self.board.executor.logistics.synapseCompiler.decodeSynapsesMapEntry(
                        currentCore, k*2)
                    synapseMapEntry = currentCore.synapseMap[k*2].discreteMapEntry
                    for l in range(synapseMapEntry.synapseLen):
                        synapse = currentCoreSynapses[l]
                        self.logger.info(
                            "Core: {}"
                            " Synapse: {}"
                            " Compartment: {}"
                            " Wgt: {}".format(
                                coreId,
                                k,
                                synapse.CIdx,
                                synapse.Wgt))
                self.logger.info("=====================================")

    def compartmentToNodeId(self, cxLocation):
        """
        Returns a mapping from CxLocation to NodeId in the graph
        Should be overwritten by the user for their mapping
        :param cxLocation: Location of the node, which is a Cx
        :return:
        """
        return cxLocation['compartmentId']

    def _preprocessForVisulaization(self):
        """
        Helper function for pre processing the graph
        Returns unpacked tuple of graph,nodes,edges,shortestPath, src, dst
        """
        edges = set()
        nodes = set()
        graph = {}
        shortestPath = []
        currentNode = None
        with open(os.path.join(self.graph, 'cluster.rpt')) as graphFile:
            for line in graphFile:
                if "node->fanout: OUT_" in line:
                    _edge = line.rstrip().split('OUT_')[1].split('_')
                    edges.add((int(_edge[0]), int(_edge[1])))
                    if int(_edge[0]) not in graph:
                        graph[int(_edge[0])] = set()
                    graph[int(_edge[0])].add(int(_edge[1]))

                if 'node: ' in line:
                    currentNode = line.rstrip().split('node: ')[1]
                    nodes.add(int(currentNode))

                if 'node->fanin: IN_' in line:
                    firstNode = line.rstrip().split(
                        'node->fanin: IN_')[1].split('_')[0]
                    edges.add((int(firstNode), int(currentNode)))
                    if int(firstNode) not in graph:
                        graph[int(firstNode)] = set()
                    graph[int(firstNode)].add(int(currentNode))

        currentNode = self.compartmentToNodeId(self.shortestPath[0])
        src = currentNode
        for node in self.shortestPath[1:]:
            shortestPath.append((currentNode, self.compartmentToNodeId(node)))
            currentNode = self.compartmentToNodeId(node)
        dst = self.compartmentToNodeId(self.shortestPath[-1])

        self.logger.debug("Graph is : {}".format(graph))
        self.logger.debug("Nodes of the graph are : {}".format(nodes))
        self.logger.debug("Edges of the graph are : {}".format(edges))
        self.logger.debug(
            "ShortestPath of the graph are : {}".format(shortestPath))
        self.logger.debug("Src of the graph are : {}".format(src))
        self.logger.debug("Dst of the graph are : {}".format(dst))

        return graph, nodes, edges, shortestPath, src, dst

    def getWaveFront(self):
        """
        Helper function that gets the wavefront of the search and the shortest path
        :return: Set of nodes, Set of edge pairs, src, dst, List of tuple (level, node1, node2) and shortest path
        """
        graph, nodes, edges, shortestPath, src, dst = self._preprocessForVisulaization()
        wavefront = []
        level = [None] * len(nodes)
        queue = deque()
        queue.append(dst)
        level[dst] = 1
        wavefront.append((level[dst], dst, dst))
        while len(queue) > 0:
            node = queue.popleft()
            if node == src:
                continue
            for connected_node in graph[node]:
                if level[connected_node] is None:
                    queue.append(connected_node)
                    level[connected_node] = level[node] + 1
                if level[connected_node] > level[node]:
                    wavefront.append(
                        (level[connected_node], node, connected_node))

        return nodes, edges, src, dst, wavefront, shortestPath
