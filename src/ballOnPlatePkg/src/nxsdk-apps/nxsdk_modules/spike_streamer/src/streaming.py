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

import os
import inspect
import numpy as np
import nxsdk.api.n2a as nx
import scipy.sparse as sps
from nxsdk.arch.n2a.n2board import N2Board

class SpikeStreamer():
    def __init__(self, net):
        """
        Create a spike streamer
        
        :param NxNet net: The network in which to place the input compartments
        """
        self.net = net
        self.snipDir = os.path.abspath(os.path.dirname(inspect.getfile(SpikeStreamer)) + "/snips")
        self.tempDir = os.path.abspath(os.path.dirname(inspect.getfile(SpikeStreamer)) + "/temp") 
        
        if not os.path.exists(self.tempDir):
            os.mkdir(self.tempDir)
        
        self.inputStreamer = False
        self.outputStreamer = False
        
    def setupSpikeInput(self, 
                        numNodes, 
                        spikesPerPacket=1024, 
                        microsecondsPerTimestep=0, 
                        logicalCoreId=None, 
                        connectionPrototype=None, 
                        compartmentPrototype=None):
        """
        :param int numNodes: How many inputs to create
        :param int spikesPerPacket: The packet size for spike injection
        :param int microsecondsPerTimestep: The maximum speed at which to run Loihi
        :param list[int] logicalCoreId: Allows controlling the placement of the input neurons
        :param ConnectionPrototype connectionPrototype: Connection properties
        :param CompartmentPrototype compartmentPrototype: Input compartment properties
        """
        self.inputStreamer = True
        self.numNodes = numNodes
        self.spikesPerPacket = spikesPerPacket
        self.microsecondsPerTimestep = microsecondsPerTimestep
        self.connectionPrototype = connectionPrototype
        self.compartmentPrototype = compartmentPrototype
        self.logicalCoreId = logicalCoreId
        
        self._makeInputLayer()
        
        self.loihiTime = 0
        self.timeReserved = int(1<<14) #special value for timer advance
        self.nothingReserved = int(1<<13) #special value for do nothing
        self.snipdata = np.full((self.spikesPerPacket*2,), self.timeReserved, dtype=int)
        self.snipIndex = 0
        self.packedPerPacket = spikesPerPacket//16
    
    
    def setupSpikeOutput(self, compartmentGroup, interval):
        
        self.outputStreamer = True
        self.outputCompartmentGroup = compartmentGroup
        self.interval = interval
        self.numOutputs = self.outputCompartmentGroup.numNodes
        
        self._setupSpikeCounters()
        
        self.numPacked=1+(self.outputCompartmentGroup.numNodes-1)//16

        
        
    def _makeInputLayer(self):
        """Create a new input layer
        """
        if self.connectionPrototype is None:
            self.connectionPrototype = nx.ConnectionPrototype(signMode=nx.SYNAPSE_SIGN_MODE.MIXED, 
                                                              weight=2,
                                                              numDelayBits=0,
                                                              numTagBits=0)
        
        if self.compartmentPrototype is None:
            self.compartmentPrototype = nx.CompartmentPrototype(vThMant=1,
                                                                compartmentVoltageDecay = 4095,
                                                                compartmentCurrentDecay = 4095)

        # create the input layer
        if self.logicalCoreId is None:
            self.inputLayer = self.net.createCompartmentGroup(size=self.numNodes, 
                                                              prototype=self.compartmentPrototype)
        else:
            self.inputLayer = self.net.createCompartmentGroup(size=self.numNodes, 
                                                              prototype=self.compartmentPrototype,
                                                              logicalCoreId=self.logicalCoreId)
            
        # create a dummy input connection. This creates the input axons our snips will send spikes to
        inStubGroup = self.net.createInputStubGroup(size=self.numNodes)

        self.inputConnectionGroup = inStubGroup.connect(self.inputLayer,
                                                        prototype = self.connectionPrototype,
                                                        connectionMask = sps.identity(self.numNodes))
        
    def configureStreamer(self, board, regenerateCoreAxon=True, interval=None, chipId=None, counterIds=None):
        """Determines the core/axon location of the model input axons and sets up snips
        which will later be used to inject spikes
        
        :param N2Board board: The compiled board object
        :param bool regenerateCoreAxon: Whether to load core/axon values from file or precompute them
        """            
            
        if self.inputStreamer is True:
            if regenerateCoreAxon is True:
                # Determine the core/axon addresses
                self.chip = [int]*self.numNodes
                self.core = [int]*self.numNodes
                self.axon = [int]*self.numNodes
                for ii, conn in enumerate(self.inputConnectionGroup):
                    (_, self.chip[ii], self.core[ii], self.axon[ii]) = self.net.resourceMap.inputAxon(conn.inputAxon.nodeId)[0]
                np.save(self.tempDir+'/axon', self.axon)
                np.save(self.tempDir+'/core', self.core)
                np.save(self.tempDir+'/chip', self.chip)
            else:
                self.axon = np.load(self.tempDir+'/axon.npy')
                self.core = np.load(self.tempDir+'/core.npy')
                self.chip = np.load(self.tempDir+'/chip.npy')
            
            streamSnip = board.createProcess(name="streaming",
                                             includeDir=self.snipDir,
                                             cFilePath=self.snipDir + "/myspiking.c",
                                             funcName="run_spiking",
                                             guardName="do_spiking",
                                             phase="spiking",
                                             lmtId=0,
                                             chipId=0)
            
            self.spikeChannel = board.createChannel(('spikeAddresses').encode(), messageSize=64, numElements=2*self.spikesPerPacket//16)
            self.spikeChannel.connect(None, streamSnip)
                
                
        if self.outputStreamer is True:
            if chipId is None:
                chipId = self.net.resourceMap.compartment(self.outputCompartmentGroup[0].nodeId)[1]
                assert self.outputCompartmentGroup.numNodes < 950, "outputs streamer supports at most 950 neurons"
            
                for comp in self.outputCompartmentGroup:
                    thisChipId = self.net.resourceMap.compartment(self.outputCompartmentGroup[0].nodeId)[1]
                    assert chipId == thisChipId, "All output compartments must lie on the same chip" 
            
            if counterIds is None:
                self.counterIds = [prb.n2Probe.counterId-32 for prb in self.probes[0].probes]
            else:
                self.counterIds = counterIds
            
            streamOutputSnip = board.createProcess("runMgmt",
                                                   includeDir=self.snipDir,
                                                   cFilePath=self.snipDir + "/runmgmt.c",
                                                   funcName="run_mgmt",
                                                   guardName="do_run_mgmt",
                                                   phase="mgmt",
                                                   lmtId=0,
                                                   chipId=chipId)
                
            self.spikeCntrChannel = board.createChannel(b'nxspkcntr', messageSize=64, numElements=1000)
            self.spikeCntrChannel.connect(streamOutputSnip, None)

        self._writeHeaderFile()
    
    
    def _writeHeaderFile(self):
        """
        Writes the temporary header file which defines constants used by snips.
        """
        
        extraHeaderFilePath = self.snipDir + '/streaming_header.h'
        f = open(extraHeaderFilePath, "w")
        f.write('/* Temporary generated file for defining parameters for input spike streaming*/\n')
        if self.inputStreamer is True:
            f.write('#define HAS_INPUT\n')
            f.write('#define SPIKES_PER_PACKET ' + str(self.spikesPerPacket)+'\n')
            f.write('#define US_PER_TIMESTEP ' + str(self.microsecondsPerTimestep)+'\n')
        if self.outputStreamer is True:
            f.write('#define HAS_OUTPUT\n')
            f.write('#define NUM_OUTPUTS ' + str(self.numOutputs)+'\n')
            f.write('#define NUM_PACKED ' + str(self.numPacked)+'\n')
            f.write('#define TIMESTEPS_PER_SAMPLE ' + str(self.interval)+'\n')
        f.close()
        
    def sendSpikes(self, spikeTargets, spikeTimes):
        """
        spikes: a list or numpy array of integers specifying which input neuron should spike
        times: a list or numpy array of monotonically increasing integers corresponding to the time at which the neuron should spike
        """
        
        for ii in range(len(spikeTimes)):
            streamTime = spikeTimes[ii]
            while(self.loihiTime<streamTime):
                self.snipIndex+=2
                self.loihiTime+=1
                if self.snipIndex == self.spikesPerPacket*2:
                    self._transmitSpikes()
            
            self.snipdata[self.snipIndex] = self.axon[spikeTargets[ii]]
            self.snipIndex+=1
            self.snipdata[self.snipIndex] = self.core[spikeTargets[ii]] | np.left_shift(self.chip[spikeTargets[ii]], 8)
            self.snipIndex+=1
            if self.snipIndex == (self.spikesPerPacket*2):
                self._transmitSpikes()
    
    def _transmitSpikes(self):
        
        snipMessage = np.left_shift(self.snipdata[1::2], 16) + np.bitwise_and(self.snipdata[0::2], (2**16)-1)
        self.spikeChannel.write(self.spikesPerPacket//16, snipMessage)
        
        self.snipdata = np.full((self.spikesPerPacket*2,), self.timeReserved, dtype=int)
        self.snipIndex = 0
        
    def flushSpikes(self):
        """
        Send any buffered spikes, even if they do not constitute a full packet.
        """
    
        while self.snipIndex < (self.spikesPerPacket*2):
            self.snipdata[self.snipIndex] = self.nothingReserved
            self.snipIndex+=1
            self.snipdata[self.snipIndex] = self.nothingReserved
            self.snipIndex+=1
        
        self._transmitSpikes()            
    
    def advanceTime(self, endTime):
        """
        Advance to a predefined time on the chip
        """
         
        while(self.loihiTime<endTime):
            self.snipIndex+=2
            self.loihiTime+=1
            if self.snipIndex == (self.spikesPerPacket*2):
                self._transmitSpikes()
        
        self.flushSpikes()
        
    def getResults(self):
        """
        Get a result from the spike streamer
        """
        results = np.array(self.spikeCntrChannel.read(self.numPacked))
        results = results[:self.numOutputs]  #remove the extras
        results = results[self.counterIds]
        return results
    
    def _setupSpikeCounters(self):
        """
        Creates dummy probes to setup the spike counters
        """
        probeCond = nx.SpikeProbeCondition(tStart=100000000)
        self.probes = self.outputCompartmentGroup.probe(nx.ProbeParameter.SPIKE, probeCond)