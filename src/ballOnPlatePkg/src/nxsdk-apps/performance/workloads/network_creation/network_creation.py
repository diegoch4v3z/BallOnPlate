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

"""Parameterized benchmark for creating large networks"""

import nxsdk.api.n2a as nx
import numpy as np
import scipy.sparse as sps

def networkCreation(numCompartments=1, numConnectionsPerCompartment=1, enableLearning=False):
    """Creates a single compartmentGroup with a single connectionGroup to itself.

    :param int numCompartments: The number of compartments to instantiate in the compartmentGroup
    :param int numConnectionsPerCompartment: The average number of connections to each compartment
    :param bool enableLearning: Determines whether the connections have a learning rule and learning enabled
    :return NxNet net: The created network
    """

    # a realistic large network may have many prototypes
    numCompartmentPrototypes = 1024
    numConnectionPrototypes = 1024
    numLearningRules = 4

    # How many connections will we have?
    numConnections = numConnectionsPerCompartment*numCompartments

    np.random.seed(147888529)  # a randomly generated seed

    # Create a network
    net = nx.NxNet()

    # Create Compartment Prototypes
    compartmentPrototype = []
    if enableLearning is True:
        enableSpikeBackprop = 1
        enableSpikeBackpropFromSelf = 1
    else:
        enableSpikeBackprop = 0
        enableSpikeBackpropFromSelf = 0

    for ii in range(numCompartmentPrototypes):
        compartmentPrototype.append(nx.CompartmentPrototype(biasMant=np.random.randint(0, 2**12-1),
                                                            biasExp=np.random.randint(
                                                                0, 6),
                                                            vThMant=np.random.randint(
                                                                0, 2**17-1),
                                                            compartmentVoltageDecay=np.random.randint(
                                                                0, 2**12-1),
                                                            enableSpikeBackprop=enableSpikeBackprop,
                                                            enableSpikeBackpropFromSelf=enableSpikeBackpropFromSelf))

    # Create an E-STDP learning rule used by the learning-enabled synapse and connect the pre synaptic spike generator.
    if enableLearning is True:
        enableLearningInt = 1
        learningRulePrototype = []
        for ii in range(numLearningRules):
            exp1 = np.random.randint(-8, 0)
            exp2 = np.random.randint(-8, 0)
            learningRulePrototype.append(net.createLearningRule(dw='2^-' + str(exp1) + '2*x1*y0 - 2^' + str(exp2) + '*y1*x0',
                                                                x1Impulse=np.random.randint(
                                                                    0, 127),
                                                                x1TimeConstant=np.random.randint(
                                                                    0, 10),
                                                                y1Impulse=np.random.randint(
                                                                    0, 127),
                                                                y1TimeConstant=np.random.randint(
                                                                    0, 10),
                                                                tEpoch=np.random.randint(0, 64)))
    else:
        enableLearningInt = 0
        learningRulePrototype = [None]*numLearningRules

    # Create Connection Prototypes
    connectionPrototypeList = []
    possibleTagBits = [-1, 0, 1, 4, 8]
    possibleWeightBits = [-1, 0, 1, 2, 3, 4, 5, 6, 8]
    for ii in range(numCompartmentPrototypes):

        weight = np.random.randint(0, 127)
        numWeightBits = possibleWeightBits[np.random.randint(
            0, len(possibleWeightBits))]
        numTagBits = possibleTagBits[np.random.randint(
            0, len(possibleTagBits))]
        signMode = np.random.randint(1, 3)

        learningRule = learningRulePrototype[np.random.randint(
            0, numLearningRules)]
        connectionPrototypeList.append(nx.ConnectionPrototype(weight=weight,
                                                              numWeightBits=numWeightBits,
                                                              numTagBits=numTagBits,
                                                              signMode=signMode,
                                                              enableLearning=enableLearningInt,
                                                              learningRule=learningRule))

    srcNeurons = np.random.randint(0, numCompartments, size=(numConnections,))

    dstNeurons = np.random.randint(0, numCompartments, size=(numConnections,))

    # connectionMask can be sparse
    connectionMask = sps.coo_matrix((np.ones(dstNeurons.shape), (dstNeurons, srcNeurons)),
                                    shape=(numCompartments, numCompartments))

    # --------------------------------------------------------------------------------
    # This matches what was used in the initial benchmark
    # --------------------------------------------------------------------------------
    weight = np.random.randint(0, 127, size=(numCompartments, numCompartments))

    # pick a prototype at random to use for all future connections until .connect() can support prototypeLists for large connections
    connectionPrototype = [
        connectionPrototypeList[np.random.randint(0, len(connectionPrototypeList))]]
    connectionPrototypeMap = sps.coo_matrix((np.zeros((numConnections,), dtype=int),
                                             (dstNeurons, srcNeurons)),
                                            shape=(numCompartments, numCompartments))
    """
    #----------------------------------------------------------------------------------------------
    # This is a new more difficult benchmark now that sparse matrices are properly supported
    #----------------------------------------------------------------------------------------------
    weight = sps.coo_matrix((np.random.randint(0,127,size=(numConnections,)), (dstNeurons, srcNeurons)),
                             shape=(numCompartments, numCompartments))
                             
    connectionPrototype = connectionPrototypeList
    connectionPrototypeMap = sps.coo_matrix((np.random.randint(0,numConnectionPrototypes,size=(numConnections,)),
                                             (dstNeurons, srcNeurons)),
                                             shape=(numCompartments, numCompartments))
    """

    # Create the compartments
    compartmentPrototypeMap = np.random.randint(
        0, numCompartmentPrototypes, size=(numCompartments,))
    cg = net.createCompartmentGroup(size=numCompartments,
                                    prototype=compartmentPrototype,
                                    prototypeMap=compartmentPrototypeMap.tolist())

    # create the connection
    cg.connect(cg,
               prototype=connectionPrototypeList,
               prototypeMap=connectionPrototypeMap,
               weight=weight,
               connectionMask=connectionMask)

    return net
