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

"""Direct Trace Injection Module"""

import os
import inspect
import nxsdk.api.n2a as nx
import numpy as np
from nxsdk.graph.processes.phase_enums import Phase

class DirectTraceInjection:
    """DirectTraceInjection allows overwriting of postsynaptic traces for a compartmentGroup.
    For now use of only 1 compartmentGroup is supported. If a user wants to write the traces of multiple compartmentGroups
    they must first be combined into a single compartmentGroup.
    """

    def __init__(self, net, compartmentGroup, enableY1=1, enableY2=0, enableY3=0):
        """Constructor for DirectTraceInjection. Specifies which compartments and traces to write

        :param NxNet net: The network to modify
        :param CompartmentGroup compartmentGroup: the compartmentGroup to modify
        :param int enableY1: boolean determining whether Y1 traces will be written
        :param int enableY2: boolean determining whether Y2 traces will be written
        :param int enableY3: boolean determining whether Y2 traces will be written
        """
        self._net = net
        self._cg = compartmentGroup
        self._enableY1 = enableY1
        self._enableY2 = enableY2
        self._enableY3 = enableY3

    def setupSnips(self, board):
        """Prepare the snips

        :param N2Board board: The compiled board to modify
        """

        numTrainCompartments = self._cg.numNodes

        # Infer which compartments to modify and the initial state of their STDP_POST_STATE register
        self._physicalCoreId, self._stdpPostStateIndex, self._stdpProfile, self._traceProfile = self._determineCoreIdCompartmentIdTraceState(
            self._cg, self._net, board)

        # write a header file to specify the snip array sizes before compilation
        # problem: this will write to the folder where the class is located
        includeDir = os.path.dirname(inspect.getfile(
            DirectTraceInjection)) + "/direct_trace_injection_snips"
        self._writeHeader(numTrainCompartments, includeDir)

        # Create a snip to run on initialization
        # This snip is used to get the index of the synapse_fmt register which must be modified
        initializationSnip = board.createSnip(
            name="runInitialization",
            includeDir=includeDir,
            cFilePath=includeDir + "/initialization.c",  # NxCore level C file
            funcName="initParams",
            guardName=None,
            phase=Phase.EMBEDDED_INIT)

        # communicates which y traces we will modify (Y1? Y2? Y3?)
        self._initWhichYChannel = board.createChannel(b'nxWhichY',
                                                      messageSize=4,
                                                      numElements=3)
        self._initWhichYChannel.connect(None, initializationSnip)

        # communicates which coreIds the compartments lie on
        # what is the limit on the channel size here?
        self._initCoreIdsChannel = board.createChannel(b'nxinitCoreIds',
                                                       messageSize=4,
                                                       numElements=numTrainCompartments)
        self._initCoreIdsChannel.connect(None, initializationSnip)

        # communicates which compartments (within the core) to modify
        # what is the limit on the channel size here?
        self._initStdpCompartmentIndexChannel = board.createChannel(b'nxinitStdpCompartmentIndex',
                                                                    messageSize=4,
                                                                    numElements=numTrainCompartments)
        self._initStdpCompartmentIndexChannel.connect(None, initializationSnip)

        # communicates the initial TraceProfile values
        self._initStdpProfileChannel = board.createChannel(b'nxinitStdpProfile',
                                                           messageSize=4,
                                                           numElements=numTrainCompartments)
        self._initStdpProfileChannel.connect(None, initializationSnip)

        # communicates which compartments (within the core) to modify
        # what is the limit on the channel size here?
        self._initTraceProfileChannel = board.createChannel(b'nxinitTraceProfile',
                                                            messageSize=4,
                                                            numElements=numTrainCompartments)
        self._initTraceProfileChannel.connect(None, initializationSnip)

        # This snip runs just before the learning phase to write the error to the postsynaptic compartment
        preLearnSnip = board.createSnip(
            name="runPreLearn",
            includeDir=includeDir,
            cFilePath=includeDir + "/prelearnmgmt.c",  # NxCore level C file
            funcName="prelearn_mgmt",
            guardName="do_prelearn_mgmt",
            phase=Phase.EMBEDDED_PRELEARN_MGMT)

        if self._enableY1 == 1:
            # what is the limit on the channel size here?
            self._traceChannelY1 = board.createChannel(b'nxY1Trace',
                                                       messageSize=4,
                                                       numElements=numTrainCompartments)
            self._traceChannelY1.connect(None, preLearnSnip)

        if self._enableY2 == 1:
            # what is the limit on the channel size here?
            self._traceChannelY2 = board.createChannel(b'nxY2Trace',
                                                       messageSize=4,
                                                       numElements=numTrainCompartments)
            self._traceChannelY2.connect(None, preLearnSnip)

        if self._enableY3 == 1:
            # what is the limit on the channel size here?
            self._traceChannelY3 = board.createChannel(b'nxY3Trace',
                                                       messageSize=4,
                                                       numElements=numTrainCompartments)
            self._traceChannelY3.connect(None, preLearnSnip)

        # store for future use
        self._numTrainCompartments = numTrainCompartments

    def _determineCoreIdCompartmentIdTraceState(self, compartmentGroup, net, board):
        """Determine the board registers which must be overwritten, as well as their initial state

        :param CompartmentGroup compartmentGroup: The group of compartments to overwrite
        :param NxNet net: the net containing the compartmentGroup
        :param N2Board board: the compiled board

        :returns:
            - physicalCoreId: the core numbers to modify
            - stdPostStateIndex: the STDP post state registers to modify
            - stdpProfile: initial values of the stdpProfile field of the STDP post state registers
            - traceProfile: initial values of the traceProfile field of the STDP post state registers
        :rtype: list(int) physicalCoreId, list(int) stdPostStateIndex, list(int) stdpProfile, list(int) traceProfile
        """

        # allocate some memory to hold the coreId and compartmentIndex
        # these values will be communicated to the snip at initialization
        physicalCoreId = [int]*compartmentGroup.numNodes
        stdpPostStateIndex = [int]*compartmentGroup.numNodes
        stdpProfile = [int]*compartmentGroup.numNodes
        traceProfile = [int]*compartmentGroup.numNodes

        # loop through the compartments and find the physical core and compartment for each
        for cc in range(compartmentGroup.numNodes):
            boarId, chipId, coreId, compartmentId, _, _ = net.resourceMap.compartment(
                compartmentGroup[cc].nodeId)
            physicalCoreId[cc] = board.n2Chips[chipId].n2Cores[coreId].id

            # the stdpPostStateIndex is the compartment entryId
            stdpPostStateIndex[cc] = int(compartmentId)
            stdpProfile[cc] = board.n2Chips[chipId].n2Cores[coreId].stdpPostState[compartmentId].stdpProfile
            traceProfile[cc] = board.n2Chips[chipId].n2Cores[coreId].stdpPostState[compartmentId].traceProfile

        return physicalCoreId, stdpPostStateIndex, stdpProfile, traceProfile

    # communicate once off at initialization the compartments to be modified

    def sendSnipInitialization(self):
        """Sends precomputed initialization data to the initialization snip
        """
        self._initWhichYChannel.write(
            3, [self._enableY1, self._enableY2, self._enableY3])

        self._initCoreIdsChannel.write(
            self._numTrainCompartments, self._physicalCoreId)

        self._initStdpCompartmentIndexChannel.write(
            self._numTrainCompartments, self._stdpPostStateIndex)

        self._initStdpProfileChannel.write(
            self._numTrainCompartments, self._stdpProfile)

        self._initTraceProfileChannel.write(
            self._numTrainCompartments, self._traceProfile)

    # this function will be called every tEpoch to write the error to the compartments

    def writeY(self, y1Values=None, y2Values=None, y3Values=None):
        """Sends the new trace values to the pre-learning management snip

        :param list(int) y1Values: The Y1 trace values to be written
        :param list(int) y2Values: The Y2 trace values to be written
        :param list(int) y3Values: The Y3 trace values to be written
        """
        if self._enableY1 == 1:
            # send the error over a channel to the prelearnmgmt snip
            self._traceChannelY1.write(self._numTrainCompartments, y1Values)

        if self._enableY2 == 1:
            # send the error over a channel to the prelearnmgmt snip
            self._traceChannelY2.write(self._numTrainCompartments, y2Values)

        if self._enableY3 == 1:
            # send the error over a channel to the prelearnmgmt snip
            self._traceChannelY3.write(self._numTrainCompartments, y3Values)

    def _writeHeader(self, numTrainCompartments, includeDir):
        """writes a header file for inclusion from snips to specify array sizes before compilation

        :param string includeDir: the directory in which to place the header file
        :param int numTrainCompartments: The number of compartments to modify traces of
        """

        # generate file with #define for number of elements
        numCompartmentsFilePath = includeDir + '/array_sizes.h'
        f = open(numCompartmentsFilePath, "w")
        f.write(
            '/* Temporary generated file for define the size of arrays before compilation */\n')
        f.write('#define num_overwrite_compartments ' +
                str(numTrainCompartments))
        f.close()
