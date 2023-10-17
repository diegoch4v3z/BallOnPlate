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

from nxsdk.net.process.spikegenprocess import SpikeGenProcess


class LineScanProcess(SpikeGenProcess):
    """
    Type of Process used to convert image into spikes. The logic here is to create the input axons.
    The actual conversion happens in the underlying SNIP linespiking.c
    """

    def __init__(self, net, numLines=480, numSegments=4):
        """
        Creates a SpikeGenProcess object
        :param net: Parent NxGraph of Process
        :param numLines: Number of lines to use for Line scanner
        :param numSegments: Number of segments a line is divided into,
                            by the linescanner. Each segment corresponds
                            to an axon.
        """
        self.numLines = numLines
        self.numSegments = numSegments
        # numPorts: Number of SpikeInputPort associated with process
        numPorts = numLines * numSegments
        super().__init__(net, numPorts)

    def __str__(self):
        return "LineScanProcess numLines {} numSegments {} numPorts {}".format(
            self.numLines, self.numSegments, self.numPorts)

    def connect(self, dst, prototype=None, prototypeMap=None, connectionMask=None, **kwargs):
        """
        Connects SpikeInputPortGroup to Compartment or CompartmentGroup
        :param dst=None: The destination Compartment/Neuron or Compartment/NeuronGroup

        :param prototype=None: A single ConnectionPrototype or list of
        ConnectionPrototype objects. If a single object is provided,
        it is used for all connections in the group. If a list is provided
        <prototypeMap> must be provided as well.
        :param prototypeMap=None: A 0/1/2D numpy array where each element
        must be within 0..len(prototype) which specifies the prototype to use
        for each connection. A non-2D array is automatically broadcast into a
        (numDstNodes x numDstNodes) array.
        :param connectionMask: A 0/1/2D numpy binary numpy array which
        specifies which connection exists within the
        (numDstNodes x numDstNodes) space. Elements in prototypeMap,
        weights, delays and tags where <connectionMask> == 0 will be ignored. A
        non-2D array is automatically broadcast into a
        (numDstNodes x numDstNodes) array.
        :param name=None: (string) A descriptive name for the group.
        :param weight: A 0/1/2D numpy array weight matrix. A non-2D array is
        automatically broadcast into a (numDstNodes x numDstNodes) array.
        :param delay: A 0/1/2D numpy array delay matrix. A non-2D array is
        automatically broadcast into a (numDstNodes x numDstNodes) array.
        :param tag: A 0/1/2D numpy array tag matrix. A non-2D array is
        automatically broadcast into a (numDstNodes x numDstNodes) array.
        :param enableLearning: A 0/1/2D numpy array enableLearning matrix. A
        non-2D array is automatically broadcast into a
        (numDstNodes x numDstNodes) array.
        :return: Connection Group Object
        """

        return super().connect(dst, prototype, prototypeMap, connectionMask, **kwargs)
