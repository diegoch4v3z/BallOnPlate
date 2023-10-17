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
# expressly stated in the License.

"""Spike Trace Injection Module"""

import os
import inspect
import nxsdk.api.n2a as nx
import numpy as np
from nxsdk.graph.processes.phase_enums import Phase

class SpikeTraceInjection:
    """SpikeTraceInjection provides means of supplying non-local information
    to learning rules throught postynaptic traces, either by directly overwriting
    a compartmentGroup's traces, or by rewiring learning synapses to auxiliary
    compartments during the learning phase.
    """
    _compartmentsPerCore = 1024

    def __init__(self, net):
        """Contructor for injecting traces into a specific network using spikes.

        :param NxNet net: The network in which to create compartmentGroups and learningRules
        """

        self._net = net

        # A list to keep track of nodeIds of compartments on the cores to be modified
        # Used later to determine which cores snips must modify
        self._cxNodeIdList = []

        # A list to keep track of the order in which rewired connections must be made
        # for each compartmentGroup
        self._connectionOrderListList = []

    def addRewiringCore(self,
                        compartmentProtoList,
                        compartmentsPerGroup=1,
                        connectionOrderList=None,
                        enableLearningList=None,
                        dwList=None, ddList=None, dtList=None,
                        **kwargs):
        """Fills a new core with compartmentGroups for which rewiring can be performed by snips.
        Returns the compartmentGroups and corresponding connectionPrototypes to use when making connections to them.

        :param list(CompartmentPrototype) compartmentProtoList: The compartmentPrototypes to use for each compartmentGroup (one per group)
        :param int compartmentsPerGroup: The number of compartments to put in each compartmentGroup. Defaults to 1
        :param list(int) connectionOrderList: The connection offsets and order in which to apply them to learning synapses for this core
        :param list(int) enableLearningList: Boolean list specifying which connectionPrototypes will have learning enabled.
        :param list(string) dwList: List of 'dw' learning rules for each compartmentGroup
        :param list(string) ddList: List of 'dd' learning rules for each compartmentGroup
        :param list(string) dtList: List of 'dt' learning rules for each compartmentGroup
        :param **kwargs: optional core parameters to pass to learning rules (trace parameters, tEpoch, etc)

        :returns:
            - cxList: Compartment groups of size 'compartmentPerGroup' with prototypes corresponding to 'compartmentProtoList'
            - connProtoList: Connection prototypes to be used when connecting to corresponding compartmentGroups
        :rtype: list(compartmentGroup), list(connectionPrototype)
        """

        # infer the number of compartmentGroups from the number of prototypes
        numCompartmentTypes = len(compartmentProtoList)

        # setup the different compartment prototypes based on the input parameter "compartmentProto"
        cxList = self._setupCompartments(compartmentsPerGroup,
                                         compartmentProtoList)

        # setup the connection prototypes
        connProtoList = self._createConnectionPrototypes(dwList, ddList, dtList,
                                                         enableLearningList,
                                                         numCompartmentTypes,
                                                         **kwargs)

        # Remember the order in which to rewire connections for this core
        self._connectionOrderListList.append(connectionOrderList)

        # Keep the nodeId of one compartment on the core so we can figure out later which physical core it got placed on
        self._cxNodeIdList.append(cxList[0][0].nodeId)

        return cxList, connProtoList

    def _setupCompartments(self,
                           compartmentsPerGroup,
                           compartmentProtoList):
        """Fills a new core with interlaced compartments based on compartmentPrototypes. 
        Returns 'n' compartmentGroups, each of which contains every 'n'th physical compartment, 
        where 'n' is the number of compartmentPrototypes passed.

        :param int compartmentsPerGroup: The size of each compartmentGroup
        :param list(compartmentPrototype) compartmentProtoList: The prototypes to use for each compartmentGroup

        :returns: compartmentGroups which can be used in the parent network
        :rtype: list(compartmentGroup)
        """
        # check that the requirements will not require more compartments than are available on a single core
        assert compartmentsPerGroup <= self._compartmentsPerCore / \
            len(compartmentProtoList), "Too many neurons on the core (>1024)"

        # Make the prototypeMap at least the size of a core
        cxprototypeMap = np.tile(np.arange(len(compartmentProtoList)), int(
            1+self._compartmentsPerCore/len(compartmentProtoList)))
        # Shorten to the number of compartments in a core
        cxprototypeMap = cxprototypeMap[:self._compartmentsPerCore]

        self._net.createCompartmentGroup(size=1,
                                         prototype=compartmentProtoList[0])

        # create dummy compartments up to a core boundary to force our compartments onto a new core
        compartmentsToNextCore = 1024 - \
            self._net._compartments.numNodes % self._compartmentsPerCore
        if compartmentsToNextCore < 1024:
            self._net.createCompartmentGroup(size=compartmentsToNextCore,
                                             prototype=compartmentProtoList[0])

        # Create a compartment group the size of a core of alternating prototypes
        cxInternal = self._net.createCompartmentGroup(size=self._compartmentsPerCore,
                                                      prototype=compartmentProtoList,
                                                      prototypeMap=cxprototypeMap.tolist())

        # allocate compartments from cxInternal into a list of compartmentGroups with one compartmentPrototype per Group
        cxList = []
        for cxIndex in range(len(compartmentProtoList)):
            cxList.append(self._net.createCompartmentGroup())
            for compartmentIndex in range(compartmentsPerGroup):
                cxList[cxIndex].addCompartments(
                    cxInternal[len(compartmentProtoList)*compartmentIndex+cxIndex])

        return cxList

    def _createConnectionPrototypes(self,
                                    dwList, ddList, dtList,
                                    enableLearningList,
                                    numCompartmentTypes,
                                    **kwargs):
        """Creates a list of connectionPrototypes to use with the compartmentGroups created by _setupCompartments

        :params list(string) dwList: list of 'dw' learning rules
        :params list(string) ddList: list of 'dd' learning rules
        :params list(string) dtList: list of 'dt' learning rules
        :params int numCompartmentTypes: the number of different compartmentGroups for which connectionPrototypes must be created
        :params **kwargs: additional core-common parameters to be passed to createLearningRule

        :returns: for use with corresponding compartmentGroups
        :rtype: list(connectionPrototypes)
        """

        # How many tag bits should we use?
        numTagBits = self._determineTagBits(dtList)

        # Create the default enableLearningList if None was passed
        enableLearningList = self._determineEnableLearningList(
            enableLearningList, numCompartmentTypes)

        # Create the default learning rules to use for those where None was passed
        dwList = self._checkDList(dwList, numCompartmentTypes)
        ddList = self._checkDList(ddList, numCompartmentTypes)
        dtList = self._checkDList(dtList, numCompartmentTypes)

        # Create the learning rule and connectionPrototype list
        connProtoList = []
        for ii in range(numCompartmentTypes):

            # If no learning rule is specified for the corresponding compartment group
            # but learning is enabled, then insert a dummy learning rule to prevent
            # it from being optimized out
            if dwList[ii] is None and ddList[ii] is None and dtList[ii] is None and enableLearningList[ii] == 1:
                dwList[ii] = 'u0*0'

            lr = self._net.createLearningRule(dw=dwList[ii],
                                              dd=ddList[ii],
                                              dt=dtList[ii],
                                              **kwargs)

            connProtoList.append(nx.ConnectionPrototype(enableLearning=enableLearningList[ii],
                                                        numTagBits=numTagBits,
                                                        learningRule=lr))
        return connProtoList

    def setupSnips(self, board):
        """Setup the initialization, pre-learning, and post-learning snips for rewiring synapses.

        :params N2Board board: the compiled board to setup snips for        
        """

        # A list of ints specifying how long the rewiring list is for each core
        self._orderLengthList = self._determineOrderLength(
            self._connectionOrderListList)

        # Define directory where SNIP C-code is located
        # How can this be done more generally
        includeDir = os.path.dirname(inspect.getfile(
            SpikeTraceInjection)) + "/spike_trace_injection_snips"

        # write the connectionOrder length and number of connections to modify to a header file for the snips
        self._writeHeader(includeDir, self._orderLengthList)

        # figure out which Synapse Format Indices on which cores are used by the learning synapses
        self._coreIds, self._synFmtIds = self._checkSynapseFormatIds(board)

        # Create the initialization snip
        #initializationSnip = board.createProcess(
        initializationSnip = board.createSnip(
            name="runInitialization",
            includeDir=includeDir,
            cFilePath=includeDir + "/initialization.c",  # NxCore level C file
            funcName="initParams",
            guardName=None,
            phase=Phase.EMBEDDED_INIT)

        # channel for specifying how long each connectionOrder list is
        self._initOrderLengthChannel = board.createChannel(b'nxConnectionOrderLengthInit',
                                                           messageSize=4,
                                                           numElements=len(self._orderLengthList))
        self._initOrderLengthChannel.connect(None, initializationSnip)

        # channel for specifying the connectionOrder lists' contents
        self._initConnectionOrderChannel = board.createChannel(b'nxConnectionOrderInit',
                                                               messageSize=4,
                                                               numElements=sum(self._orderLengthList))
        self._initConnectionOrderChannel.connect(None, initializationSnip)

        # channel for specifying which synapse format registers each connectionOrder list corresponds to
        self._initSynFmtChannel = board.createChannel(b'nxSynFmtinit',
                                                      messageSize=4,
                                                      numElements=len(self._orderLengthList))  # assume only one synFmtId/core
        self._initSynFmtChannel.connect(None, initializationSnip)

        # channel for specifying which core each connectionOrder list corresponds to
        self._initCoreIdChannel = board.createChannel(b'nxCoreIdinit',
                                                      messageSize=4,
                                                      numElements=len(self._orderLengthList))  # assume only one synFmtId/core
        self._initCoreIdChannel.connect(None, initializationSnip)


        # Runs just before the learning phase to rewire the synapses
        preLearnSnip = board.createSnip(
            name="runPreLearn",
            includeDir=includeDir,
            cFilePath=includeDir + "/prelearnmgmt.c",  # NxCore level C file
            funcName="prelearn_mgmt",
            guardName="do_prelearn_mgmt",
            phase=Phase.EMBEDDED_PRELEARN_MGMT)

        # Runs just after the learning phase to wire the synapses back
        postLearnSnip = board.createSnip(
            name="runPostLearn",
            includeDir=includeDir,
            cFilePath=includeDir + "/postlearnmgmt.c",  # NxCore level C file
            funcName="postlearn_mgmt",
            guardName="do_postlearn_mgmt",
            phase=Phase.EMBEDDED_MGMT)

    def sendSnipInitialization(self):
        """Sends initialization information over the snip channels"""

        # communicate the Synapse Format Indices to the initialization SNIP
        self._initSynFmtChannel.write(
            len(self._orderLengthList), self._synFmtIds)

        # communicate the coreIds to the initialization SNIP
        self._initCoreIdChannel.write(
            len(self._orderLengthList), self._coreIds)

        # communicte how long each connectionOrder list is
        self._initOrderLengthChannel.write(
            len(self._orderLengthList), self._orderLengthList)

        # communicate the connectionOrder lists themselves
        for connectionOrder in self._connectionOrderListList:
            self._initConnectionOrderChannel.write(
                len(connectionOrder), connectionOrder)

    def _checkSynapseFormatIds(self, board):
        """Determines which coreIds and SynFmtIds the snips need to modify

        :params N2board board: The board with placed registers to modify

        :returns:
            - coreIds: The coreIds to modify
            - synFmtIds: The synFmtIds to modify
        :rtype: list(int), list(int)
        """

        coreIds = []
        synFmtIds = []

        # loop through the compartment nodes which were stored during core setup
        for compartmentNodeId in self._cxNodeIdList:
            # Figure out which chipID, coreId the compartments were placed on
            _, chipId, coreId, _, _, _ = self._net.resourceMap.compartment(
                compartmentNodeId)
            coreIds.append(board.n2Chips[chipId].n2Cores[coreId].id)

            # Check that all learning synapses on the core use the same synFmt register
            # and that no non-learning synapses use that synFmt register
            numLearningSynFmtIds = 0
            for learningSynFmtId in range(1, 16):
                # find or add an enum somewhere for the meaning of learningCfg... 1 means learning enabled
                if board.n2Chips[chipId].n2Cores[coreId].synapseFmt[learningSynFmtId].learningCfg == 1:
                    numLearningSynFmtIds = numLearningSynFmtIds+1
                    synFmtIds.append(learningSynFmtId)

            assert numLearningSynFmtIds == 1, "Too many different learning synapse formats on core " + \
                str(coreId)

        return coreIds, synFmtIds

    def _determineTagBits(self, dtList):
        """Determines how many tag bits the core should use. If dtList is specified, the maximum number of bits is allocated, 
        otherwise no bits are allocated.

        :param list(string) dtList: list of 'dt' learning rules

        :returns: How many tag bits the core should use
        :rtype: int
        """
        # here we assume either no tag, or maximum tag
        if dtList is None:
            numTagBits = 0
        else:
            numTagBits = 8

        return numTagBits

    def _writeHeader(self, includeDir, orderLengthList):
        """Creates a temporary header file for snips to define array sizes before C compilation

        :param string includeDir: the directory in which to place the header file
        :param list(int) orderLengthList: a list of how long the connectionOrder list is for each core 
        """
        # how many cores must we modify?
        numCores = len(orderLengthList)

        # what is the lenght of the longest connectionOrder list?
        maxOrderLength = max(orderLengthList)

        numConnectionsFile = includeDir + '/array_sizes.h'
        f = open(numConnectionsFile, "w")
        f.write(
            '/* Temporary generated file for define the size of arrays before compilation */\n')
        f.write('#define num_rewiring_cores ' + str(numCores) + '\n')
        f.write('#define max_connection_order_length ' + str(maxOrderLength))
        f.close()

    def _determineEnableLearningList(self, enableLearningList, numCompartmentTypes):
        """Creates a default list of which synapses learning should be enabled for if None was passed.
        Default is to only enable learning for synapses of the first compartmentGroup

        :param list(int) enableLearningList: logical list specifying which compartmentGroup's synapses should have learning enabled
        :param int numCompartmentTypes: How many compartmentPrototypes are in use

        :returns: List if CompartmentGroups with learning enabled
        :rtype: list(int)
        """

        if enableLearningList is None:
            enableLearningList = [1] + [0]*(numCompartmentTypes-1)

        return enableLearningList

    def _checkDList(self, dList, numCompartmentTypes):
        """Placeholder for determining the number of tag bits to use. For now it uses the maximum if Dlist is not None.

        :params list(string) dList: A list of learning rules (dw or dt or dd)
        :params int numCompartmentTypes: How many compartmenttypes (and therefore learning rules) the core will use

        :returns: A list of learning rules (dw or dt or dd)
        :rtype: list(string)
        """

        # if dList is None, make it a list of None's instead
        if dList is None:
            dList = [None]*numCompartmentTypes
        return dList

    def _determineOrderLength(self, connectionOrderListList):
        """Determines how long the connectionOrderList is for each core to be modified

        :param list(list(int)) connectionOrderListList: holds the lists of connectionOrders for each core

        :returns: The length of each connectionOrderList
        :rtype: list(int)
        """

        # how long is the order list for each core?
        orderLength = []
        for orderList in connectionOrderListList:
            orderLength.append(len(orderList))

        return orderLength
