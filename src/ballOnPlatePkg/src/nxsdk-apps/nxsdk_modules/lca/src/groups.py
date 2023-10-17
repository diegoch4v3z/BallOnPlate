###############################################################
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
###############################################################

import math
from collections import OrderedDict
from nxsdk.arch.n2a.graph.n2acore import N2ACore


class NeuronGroup(object):
    """Represents a group of neurons that is connected to other neuronGroups
    via connectivityGroups. A NeuronGroup itself consists of multiple
    neuronGroupsSlices."""

    def __init__(self, id, neuronIds):
        """Initializes a neuronGroup.

        :param id: (int) Unique id of neuronGroup.
        :param neuronIds: (list<int>/range) Globally unique ids of neurons
                          assigned to this neuronGroup.
        """
        self.id = id
        self.neuronIds = neuronIds
        self.neuronGroupSlices = []
        self.inputConnGroups = OrderedDict()
        self.outputConnGroups = OrderedDict()

    @property
    def numInputConnections(self):
        """Returns the number of input connectivityGroups that connect to
        this neuronGroup."""

        return len(self.inputConnGroups)

    @property
    def numOutputConnections(self):
        """Returns the number of output connectivityGroups connecting from
        this neuronGroup."""

        return len(self.outputConnGroups)

    @property
    def numNeurons(self):
        """Returns number of neurons in neuronGroup."""
        return len(self.neuronIds)

    @property
    def numSlices(self):
        """Returns the number of neuronGroupsSlices"""
        return len(self.neuronGroupSlices)

    def addInputConnection(self, connGrp):
        """Adds an input connectivityGroup that connects to this
        neuronGroup."""

        self.inputConnGroups[connGrp.id] = connGrp

    def addOutputConnection(self, connGrp):
        """Adds an output connectivityGroup that connects from this
        neuronGroup."""

        self.outputConnGroups[connGrp.id] = connGrp

    def slice(self, neuroCores):
        """Slices neuronGroup over multiple neuroCores. Neurons are
        distributed as evenly as possible over all cores to keep runtimes
        about equal per core."""

        # Initialize temporary variables
        sliceId = 0
        numSlices = len(neuroCores)
        numNeuronsPerSlice = math.ceil(len(self.neuronIds)/numSlices*1.0)
        nCtr = 0
        for c in neuroCores:
            # Select neuronIds for new slice from neuronGroups neuronIds
            if sliceId < numSlices-1:
                idx1 = sliceId*numNeuronsPerSlice
                idx2 = (sliceId+1)*numNeuronsPerSlice
                nIds = self.neuronIds[idx1:idx2]
            else:
                # Last slice gets neurons that are left
                nIds = self.neuronIds[sliceId*numNeuronsPerSlice:]
            # Create new slice and add to neuronGroup
            ngs = NeuronGroupSlice(sliceId, self, c, nIds, nCtr)
            nCtr += len(nIds)
            self.neuronGroupSlices.append(ngs)
            sliceId += 1


class NeuronGroupSlice(object):
    """Represents a subset of a neuronGroup or those neurons of a
    neuronGroup that is allocated to a particular neuroCore together with
    slices of other neuronGroups."""

    def __init__(self, sliceId, neuronGroup, neuroCore, neuronIds,
                 numPreviousNeurons):
        """Initializes neuronGroupSlice.

        :param sliceId: (int) Unique id of slice relative to neuronGroup.
        :param neuronGroup: (NeuronGroup) Parent that slice belongs to.
        :param neuroCore: (NeuroCore) N2Core that slice is assigned to.
        :param neuronIds: (list<int>/range) Globally unique neuronIds in
                          this slice.
        :param numPreviousNeurons: (int) Number of neurons in prior slices
                                   of parent neuronGroup.
        """
        self.id = sliceId
        self.neuronGroup = neuronGroup
        self.neuroCore = neuroCore
        self.neuronIds = neuronIds
        self.numPreviousNeurons = numPreviousNeurons

        self.connGrpToSynIdMap = OrderedDict()
        self.connGrpToInputAxonIdMap = OrderedDict()
        self.compartmentIds = None
        self.connGrpToOutputAxonIdMap = OrderedDict()

    @property
    def numNeurons(self):
        """Returns number of neurons in neuronGroupSlice."""

        return len(self.neuronIds)

    def getRelativeNeuronIds(self):
        """Returns relative neuronIds with respect to neuronGroup the
        neuronGroupsSlice is a member of."""

        return range(self.numPreviousNeurons,
                     self.numPreviousNeurons+self.numNeurons)

    def addConnGrpToAxonToSynIdMap(self, cg, axonToSynIdMap):
        """Adds key/value pair to dict mapping connectivityGroup ->
        axonToSynIdMap."""
        self.connGrpToSynIdMap[cg] = axonToSynIdMap

    def addConnGrpToInputAxonIdMap(self, cg, popAxonId):
        """Adds key/value pair to dict mapping connectivityGroup ->
        populationInputAxon. There is only a single population input axon
        for each connectivityGroup."""
        self.connGrpToInputAxonIdMap[cg] = popAxonId

    def addCompartmentIds(self, cxIds):
        self.compartmentIds = cxIds

    def addConnGrpToOutputAxonIdMap(self, cg, toNg, outAxonIds):
        """Adds key/value pair to dict mapping connectivityGroup ->
        toNg -> outputAxonIds. There are multiple output axons for each
        connectivityGroup to connect to each neuronGroupSlice."""

        if cg in self.connGrpToOutputAxonIdMap:
            self.connGrpToOutputAxonIdMap[cg][toNg] = outAxonIds
        else:
            self.connGrpToOutputAxonIdMap[cg] = {toNg: outAxonIds}

    def getInputAxonId(self, inConnGrp):
        """Returns the input axonId for an input connectivityGroup on a
        neuroCore."""

        assert inConnGrp in self.connGrpToInputAxonIdMap, \
            'No input axon for connectivityGroup[id=%d]' % (inConnGrp.id)
        return self.connGrpToInputAxonIdMap[inConnGrp]

    def getAxonToSynIdMap(self, inConnGrp):
        """Returns the the axonToSynIdMap for the given
        input connectivityGroup."""

        assert inConnGrp in self.connGrpToSynIdMap, \
            'No axonToSynIdMap for connectivityGroup[id=%d]' % (inConnGrp.id)
        return self.connGrpToSynIdMap[inConnGrp]

    def getOutputAxonIds(self, outConnGrp, toNg):
        """Returns the output toNg -> outAxonIds map for an output
        connectivityGroup on a neuroCore. There is one outputAxonId for each
        output slice."""

        assert outConnGrp in self.connGrpToOutputAxonIdMap, \
            'No outputAxonIds for connectivityGroup[id=%d]' % (outConnGrp.id)
        assert toNg in self.connGrpToOutputAxonIdMap[outConnGrp], \
            'No outputAxonIds for toNeuronGroup[id=%d] for connectivityGroup[' \
            'id=%d]' % (toNg.id, outConnGrp.id)

        return self.connGrpToOutputAxonIdMap[outConnGrp][toNg]


class ConnectivityGroup(object):
    """Represents a shared connectivityGroup used to connect pairs of
    neuronGroups."""

    def __init__(self, id):
        self.id = id
        self.forwardConnections = OrderedDict()
        self.backwardConnections = OrderedDict()

    def connect(self, ng1, ng2):
        """Connects two neuronsGroups with this connectivityGroup.
        This also creates a map from a connectivityGroup to all """
        if ng1 in self.forwardConnections.keys():
            self.forwardConnections[ng1].append(ng2)
        else:
            self.forwardConnections[ng1] = [ng2]
        if ng2 in self.backwardConnections.keys():
            self.backwardConnections[ng2].append(ng1)
        else:
            self.backwardConnections[ng2] = [ng1]
        ng1.addOutputConnection(self)
        ng2.addInputConnection(self)

    def getToNeuronGroups(self, fromNg):
        """Returns all neuronGroups that fromNg connects to via this
        connectivityGroup."""
        return self.forwardConnections[fromNg]

    def getFromNeuronGroups(self, toNg):
        """Returns all neuronGroups that connect to toNg via this
        connectivityGroup."""
        return self.backwardConnections[toNg]

    def printForwardConnections(self):
        """Prints the forwardConnections of connectivityGroup."""

        print('ConnectivityGroup[id=%d] forwardConnections:' % (self.id))
        for fromNg, toNgs in self.forwardConnections.items():
            print('fromNg: %d -> toNg: ' % (fromNg.id), end='')
            for toNg in toNgs:
                print('%d ' % (toNg.id), end='')
            print()

    def printBackwardConnections(self):
        """Prints the backwardConnections of connectivityGroup."""

        print('ConnectivityGroup[id=%d] backwardConnections:' % (self.id))
        for toNg, fromNgs in self.backwardConnections.items():
            print('fromNg: ', end='')
            for fromNg in fromNgs:
                print('%d ' % (fromNg.id), end='')
            print('-> toNg: %d' % (toNg.id))


class NeuroCore(object):
    """Represents a neuroCore. It is only a virtual container for mapping
    tables."""

    def __init__(self, coreId, cxSpacing=1):
        self.id = coreId
        self.cxSpacing = cxSpacing
        self.connGrpMap = OrderedDict()
        self.numSynapses = 0
        self.numInputAxons = 0
        self.firstCompartment = 0
        self.numCompartments = 0
        self.numAxonCfgEntries = 0
        self._n2Core = None

    @property
    def n2Core(self):
        assert self._n2Core is not None, "N2Core is not assigned yet."
        return self._n2Core

    @property
    def connectionGroups(self):
        return self.connGrpMap.keys()

    @n2Core.setter
    def n2Core(self, val):
        assert isinstance(val, N2ACore), "Assigned value must be of type " \
            "N2Core."
        self._n2Core = val

    def hasConnectivityGroup(self, connGrp, relativeNeuronIds):
        """Returns True if neuroCore contains connectivityGroup.

        :param connGrp: (ConnectivityGroup) The connectivityGroup to check.
        :param relativeNeuronIds: (list<int>/range) The relativeNeuronIds we
        expect the connectivityGroup to map to.
        """

        if connGrp in self.connGrpMap:
            assert set(self.connGrpMap[connGrp][1]) == \
                set(relativeNeuronIds), \
                "NeuroCore contains connectivityGroup but relativeNeuronIds " \
                "don't match."
            return True
        else:
            return False

    def getAxonToSynIdMap(self, connGrp):
        """Returns the axonToSynIdMap for given connectivityGroup, that is a
        map from relative axonIds or relative neuronIds with respect to a
        neuronGroup to the associated synapseIds.

        :param connGrp: (ConnectivityGroup) The connectivityGroup for which
        we want to get the axonToSynIdMap.
        """

        return self.connGrpMap[connGrp][0]

    def getRelativeNeuronIds(self, connGrp):
        """Returns the relativeNeuronIs for given connectivityGroup.

        :param connGrp: (ConnectivityGroup) The connectivityGroup for which
        we want to get the axonToSynIdMap."""

        return self.connGrpMap[connGrp][1]

    def allocateSynapses(self, connGrp, numNeuronsPerGroup, relativeNeuronIds):
        """Allocates new synapseIds for connectivityGroup.

        :param connGrp: (ConnectivityGroup) The connectivityGroup for which
                        to allocate the synapses to that we can check whether
                        reusable synaptic resources for a particular
                        connectivityGroup are already allocated.
        :param numNeuronsPerGroup: (int) The number of virtual input axons
                                   targeting the connections of this
                                   connectivityGroup.
        :param relativeNeuronIds: (list<int>/range) The relative neuronIds
                                  for which this core contains synaptic
                                  resources.
        :returns axonToSynIdMap: (dict) Maps relativeNeuronId with respect
                                 to neuronGroup to synapseIds.
        """

        axonToSynIdMap = OrderedDict()
        numSynPerAxon = len(relativeNeuronIds)
        for i in range(0, numNeuronsPerGroup):
            # Generate new unique synapseIds
            synIds = range(self.numSynapses, self.numSynapses+numSynPerAxon)
            # Update map from relativeNeuronId to synIds
            axonToSynIdMap[i] = synIds
            self.numSynapses += len(synIds)

        # Update map from connectivityGroup to its associated synapses
        self.connGrpMap[connGrp] = (axonToSynIdMap, relativeNeuronIds)

        return axonToSynIdMap

    def allocateInputAxons(self, numNewInputAxons):
        """Allocates new input axonsIds/synMapIds."""
        assert numNewInputAxons == 1, "Cannot allocate more than one input " \
            "axon at once."
        # axonIds = range(self.numInputAxons, self.numInputAxons+numNewInputAxons)
        axonIds = self.numInputAxons
        self.numInputAxons += numNewInputAxons
        assert self.numInputAxons <= 2**12, 'Exceeding maximum number of ' \
                                            'input axons: numInputAxons=%d > ' \
                                            '2**12' % (self.numInputAxons)
        return axonIds

    def allocateCompartments(self, numNewCompartments):
        """Allocates new compartments.
        If cxSpacing==1, compartments are allocated sequentially.
        If cxSpacing>1, compartments are allocated sequentially but
        subsequent compartment ids are spaced by cxSpacing.

        :param numNewCompartments: (int) Number of new compartments to
        allocate.
        """

        if self.cxSpacing == 1:
            cxIds = range(self.numCompartments,
                          self.numCompartments + numNewCompartments)
        else:
            assert self.firstCompartment < self.cxSpacing, \
                'Cannot add any more compartments. cxSpacing=%d.'\
                % (self.cxSpacing)
            cxIds = range(self.firstCompartment,
                          self.cxSpacing*numNewCompartments,
                          self.cxSpacing)
            self.firstCompartment += 1
        self.numCompartments += numNewCompartments
        assert self.numCompartments <= 2**10, \
            'Exceeding maximum number of compartments: ' \
            'numCompartments=%d > 2**12' % (self.numCompartments)
        return cxIds

    def allocateOutputAxons(self, numNewOutputAxons, axonCfgSpacing=1):
        """Allocates new output axonsIds."""

        axonCfgIds = range(self.numAxonCfgEntries,
                           self.numAxonCfgEntries + numNewOutputAxons*axonCfgSpacing,
                           axonCfgSpacing)
        self.numAxonCfgEntries += numNewOutputAxons * axonCfgSpacing
        assert self.numAxonCfgEntries <= 2 ** 12, 'Exceeding maximum number of ' \
            'axonCfg entries: ' \
            'numAxonCfgEntries=%d > ' \
            '2**12' % (self.numAxonCfgEntries)
        return axonCfgIds

    def printInputAxons(self):
        """Debug feature to print input axons"""
        for i in range(0, self.numInputAxons):
            print(self.n2Core.synapseMap[i].popuplation32MapEntry)
