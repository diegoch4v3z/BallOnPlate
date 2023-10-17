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

"""KNN module for Loihi"""

import numpy as np
from nxsdk.arch.n2a.n2board import N2Board
import sys
import os 
import inspect
import fileinput
import time

class Knn:
    """An approximate KNN search algorithm for Loihi"""
    def __init__(self, dataset, threshold=64000, k=100, queryDuration=120, spikesPerQuery=240, benchmark=False):
        """
        Creates a KNN system on Loihi which can be queried.
        
        :param ndarray dataset: 2D array of dataset samples, one per row
        :param int threshold: The neuron threshold to use
        :param int k: The number of top matches to return
        :param int queryDuration: Controls the temporal precision of spike encoding
        :param int spikesPerQuery: The number of most significant dimensions to encode as spikes
        
        """
        self.neuronsPerChipKnn = 2400
        self.numCountersPerLakemont = 800
        self.neuronsPerCoreKnn = 32
        self.neuronsPerCoreRouting = 32
        self.inputDimension = 500
        self.neuronStartCoreKnn = 0
        self.pipelining = 64
        self.neuronsPerSumCounter = 128
        self.remoteMessageByteSize = 8
        self.localMessageByteSize = 32
        self.prevEndTime = 0
        
        
        # some benchmarking accumulators
        self.resultProcTime = 0
        self.timingProcTime = 0
        self.resultRecTime = 0
        self.timingRecTime = 0
        self.queryPreProcTime = 0
        self.querySendTime = 0
        
        self.runTime = 0
        
        self.benchmark = benchmark
        self.queryDuration = queryDuration
        if spikesPerQuery is None:
            self.spikesPerQuery = self.inputDimension
        else:
            self.spikesPerQuery = min(self.inputDimension, spikesPerQuery)
            
        assert k<=100, "k is only supported up to 100"
        self.k = k
        assert threshold<(1<<17), "Threshold must be below 2^17"
        self.neuronThreshold = threshold
        assert 2*self.inputDimension < 1024, "only up to 1023 inputs allowed"
        self.neuronsPerChipRouting = self.neuronsPerCoreRouting*((2*self.inputDimension+self.neuronsPerCoreRouting-1)//self.neuronsPerCoreRouting)
        
        self.tokensPerPacket = int(32*((self.spikesPerQuery + np.ceil(self.queryDuration/63) + 31)//32))
        self.numKnnChips = (dataset.shape[0]+self.neuronsPerChipKnn-1)//self.neuronsPerChipKnn
        self.numChips = 32*((self.numKnnChips+31)//32)

        self.knnCoresPerChip = (self.neuronsPerChipKnn + (self.neuronsPerCoreKnn-1))//self.neuronsPerCoreKnn
        assert self.neuronsPerChipRouting/self.neuronsPerCoreRouting + self.knnCoresPerChip <= 128, "Parameters require too many cores per chip"
        
        self._createBoard(self.numChips)
        snipDirectory = os.path.abspath(os.path.dirname(inspect.getfile(Knn)) + "/snips")
        self._setupSnips(snipDirectory)
        self._encodeWeights(dataset)
        self.board.start()

    def _createBoard(self, numChips):
        """
        Creates the board object
        """
        MAX_CORES = 128
        self.board = N2Board(id=1, numChips=numChips, numCores=[MAX_CORES] * numChips)

    def _setupSnips(self, snipDirectory):
        """
        Declares snips for every Lakemont and sets up the channels.
        """
        self._writeParams(snipDirectory)
        NUM_CPUS_PER_CHIP = 3
        for chip in range(self.numChips):
            for ii in range(NUM_CPUS_PER_CHIP):
                reconfigurationProcess = self.board.createProcess(name="recon",
                                    includeDir=snipDirectory,
                                    cFilePath="{}/reconfigure.c".format(snipDirectory),
                                    funcName="reconfigure",
                                    guardName="doReconfigure",
                                    phase="init",
                                    lmtId=ii,
                                    chipId=chip)

                spikeInputProcess = self.board.createProcess(name="spike",
                                    includeDir=snipDirectory,
                                    cFilePath="{}/spiking.c".format(snipDirectory),
                                    funcName="spiking",
                                    guardName="doSpiking",
                                    phase="spiking",
                                    lmtId=ii,
                                    chipId=chip)

                resetProcess = self.board.createProcess(name="reset",
                            includeDir=snipDirectory,
                            cFilePath="{}/reset_query.c".format(snipDirectory),
                            funcName="reset",
                            guardName="do_reset",
                            phase="remoteMgmt",
                            lmtId=ii,
                            chipId=chip)

                if ii == 0 and chip == 0:
                    INLET_PROCESS = spikeInputProcess

                if ii == 0 and chip == self.numChips - 1:
                    OUTLET_PROCESS = spikeInputProcess

                if ii==0 and chip == 1:
                    BENCHMARK_PROCESS = resetProcess
                    
        premptProcess = self.board.createProcess(name="prempt",
                            includeDir=snipDirectory,
                            cFilePath="{}/reset_mgmt.c".format(snipDirectory),
                            funcName="run_mgmt",
                            guardName="do_run_mgmt",
                            phase="mgmt",
                            lmtId=0,
                            chipId=self.numChips-1)

        self.spikeChannel = self.board.createChannel('spikeAddresses', messageSize=64, numElements=1024, slack=16)
        self.spikeChannel.connect(None, INLET_PROCESS)

        self.resultChannel = self.board.createChannel('results', messageSize=64, numElements=1024, slack=16)
        self.resultChannel.connect(OUTLET_PROCESS, None)
        
        if self.benchmark:
            self.timingChannel = self.board.createChannel('timingInfo', messageSize=64, numElements=1024, slack=16)
            self.timingChannel.connect(BENCHMARK_PROCESS, None)

        
    def _writeParams(self, snipDirectory):
        """
        Writes a parameter header file used by the snips.
        """
        
        
        
        paramDict = {'TOKENS_PER_PACKET': self.tokensPerPacket,
                     'K' : self.k,
                     'THRESHOLD_KNN' : self.neuronThreshold,
                     'NEURONS_PER_CHIP_KNN': self.neuronsPerChipKnn,
                     'NEURONS_PER_CORE_KNN': self.neuronsPerCoreKnn,
                     'NEURON_START_CORE_KNN': self.neuronStartCoreKnn,
                     'NEURONS_PER_CHIP_ROUTING': self.neuronsPerChipRouting,
                     'NEURONS_PER_CORE_ROUTING': self.neuronsPerCoreRouting,
                     'NEURONS_PER_SUM_COUNTER': self.neuronsPerSumCounter,
                     'REMOTE_MESSAGE_BYTE_SIZE': self.remoteMessageByteSize,
                     'LOCAL_MESSAGE_BYTE_SIZE': self.localMessageByteSize
                    } 

        for line in fileinput.input("{}/params.h".format(snipDirectory), inplace=True):
            for string, value in paramDict.items():
                if line.strip().startswith('#define {} '.format(string)):
                    line = '#define {} {}\n'.format(string, value)
                if line.strip().startswith('#define BENCHMARK'):
                    line = ''
            sys.stdout.write(line)
        if self.benchmark:
            os.system('sed -i \'20 a #define BENCHMARK\' {} '.format("{}/params.h".format(snipDirectory)))
            
            
    def _encodeWeightList(self, w, num64BitWordsPerList):
        """
        Encodes a single synaptic list.
        """
        headerBits = 4
        headerVal = 1
        # generate list bits
        synMemList = headerVal
        for idx, weight in enumerate(w):
            synMemList |= ((np.asscalar(weight) & ((1<<8)-1)) << (headerBits+8*idx))
        # break list up into 64 bit words
        synMemWords = np.zeros((num64BitWordsPerList,), dtype=np.uint64)
        for ii in range(num64BitWordsPerList):
            synMemWords[ii] = np.int(synMemList & ((1<<64)-1))
            synMemList = synMemList>>64
        synMemWords = synMemWords.astype(np.uint)
        return synMemWords

    def _encodeWeights(self, dataset):
        """
        Converts dataset to synaptic weights and encodes the weights for Loihi
        """
        numNeurons = dataset.shape[0]
        headerBits = 4
        numInputNeurons = self.inputDimension*2
        assert dataset.shape[1] == self.inputDimension, "input dataset must be of shape (n,{})".format(self.inputDimension)
        
        wAll = dataset.copy()
        wAll -= np.expand_dims(wAll.mean(axis=1), axis=1)
        wAll /= np.expand_dims(np.linalg.norm(wAll, axis=1), axis=1) + np.finfo(float).eps
        wAll *= 255/np.max(np.abs(wAll[:]))
        wAll = np.concatenate((wAll, -wAll), axis=1).astype(int)
        wAll = wAll>>1
        numBitsTotal = headerBits+8*self.neuronsPerCoreKnn
        num64BitWordsPerList = np.ceil(numBitsTotal/64).astype(int)
        num64BitWordsTotal = num64BitWordsPerList * numInputNeurons

        for chip in range(self.numKnnChips):
            for core in range(self.knnCoresPerChip):
                synMemWords = np.zeros((num64BitWordsTotal,), dtype=np.uint)
                wCore = wAll[(self.neuronsPerChipKnn*chip+core*self.neuronsPerCoreKnn):(self.neuronsPerChipKnn*chip+(core+1)*self.neuronsPerCoreKnn), :]
                #wCore = wAll[(core*self.neuronsPerCoreKnn):((core+1)*self.neuronsPerCoreKnn), :] # To implement the same weights on every chip
                for inputAxon in range(numInputNeurons):
                    wEntry = wCore[:, inputAxon]
                    synMemWords[(inputAxon*num64BitWordsPerList):((inputAxon+1)*num64BitWordsPerList)] = self._encodeWeightList(wEntry, num64BitWordsPerList)

                knnCore = self.board.n2Chips[chip].n2Cores[self.neuronStartCoreKnn + core]

                for k in range(len(synMemWords)):
                    knnCore.synapseMem[k].dWord = synMemWords[k]

    def _interpretResult(self, q):
        """
        Interprets search results returned by the board
        """
        q = np.array(q)
        q = np.vstack((q, q>>16)).T.flatten()&((1<<16)-1)
        numSolutionsExpected = q[1]
        results       = np.zeros((numSolutionsExpected,), dtype=int)
        solutionTimes = np.zeros((numSolutionsExpected,), dtype=int)
        
        chip = 0
        cpu = 0
        resultIndex = 0
        
        terminalToken = (1<<16)-1
        expectHeader = True
        idx = 2
        while resultIndex < numSolutionsExpected:
            word = q[idx]
            idx+=1
            if word == terminalToken:
                expectHeader = True
            else:
                if expectHeader:
                    time = (word>>8)
                    chip = word & ((1<<8)-1)
                    expectHeader = False
                else:
                    spikeCounter = word>>2
                    cpu = word & ((1<<2)-1)
                    results[resultIndex] = chip*self.neuronsPerChipKnn + cpu*self.numCountersPerLakemont + spikeCounter
                    solutionTimes[resultIndex] = time
                    resultIndex+=1
        
        time = 0
        for ii in range(4):
            time |= q[idx+ii]<<(16*ii)
        
        return results, solutionTimes, (time/400e6)
        
    def _interpretTiming(self, t):
        """
        Interprets timing information returned by the board
        """
        t = np.array(t)
        numSteps = (t[0]>>8) & ((1<<8)-1)
        numMessages = (t[0]) & ((1<<8)-1)
        
        t = np.vstack((t, t>>16)).T.flatten()&((1<<16)-1)
        t[0] = int(self.prevEndTime)
        self.prevEndTime = t[numSteps]
        timePerTimestep = np.diff(t[:numSteps+1])
        timePerTimestep[timePerTimestep<=0] += 1<<16
        timePerTimestep = timePerTimestep*64/400 # in microseconds
        return timePerTimestep, numSteps

    def _sendQuery(self, query):
        """
        Sends a query to the board
        """
        tStart = time.time()
        #self.spikeArray[:] = ((63<<10)|1022)
        spikeTimes, spikeIdx = self._encodeSpikes(query)
        
        spikeTimes = np.insert(np.diff(spikeTimes),0,0) # newer versions of numpy can use "prepend"
        while spikeTimes.max()>63:
            idxs = np.where(spikeTimes>63)[0]
            spikeTimes[idxs] -= 63
            spikeTimes = np.insert(spikeTimes,idxs,63)
            spikeIdx = np.insert(spikeIdx,idxs,1023)
            
        self.spikeArray[:spikeTimes.shape[0]] = (spikeTimes<<10) | spikeIdx
        self.spikeArray[spikeTimes.shape[0]:] = ((63<<10)|1022)
        
        rawSpikeArray = self.spikeArray[::2] | (self.spikeArray[1::2]<<16)
        rawSpikeArray[rawSpikeArray>=(1<<31)] -= (1<<32)
        tEnd = time.time()
        self.queryPreProcTime += tEnd - tStart
        
        tStart = time.time()
        self.spikeChannel.write(self.tokensPerPacket//32, rawSpikeArray)
        tEnd = time.time()
        self.querySendTime += tEnd - tStart
        
    def _getResult(self, query, index):
        """
        Receives query result and timing information
        """
        tStart = time.time()
        rawResults = self.resultChannel.read(1)
        numMessages = rawResults[0] & ((1<<16)-1)
        rawResults.extend(self.resultChannel.read(numMessages-1))
        tEnd = time.time()
        self.resultRecTime += tEnd-tStart
        
        tStart = time.time()
        thisResult, thisTiming, thisQueryTime = self._interpretResult(rawResults)
        
        ## to tie break all results
        #self.results[index,:] = thisResult[np.argpartition(np.dot(self.rawData[thisResult,:], query), -self.k)[-self.k:]]
        # to tie break only tied for k-th place
        clearIndices = thisResult[thisTiming!=thisTiming[-1]]
        tiedIndices = thisResult[thisTiming==thisTiming[-1]]
        numTiedWinners = self.k-clearIndices.shape[0]
        tiedIndices = tiedIndices[np.argpartition(np.dot(self.rawData[tiedIndices,:], query), -numTiedWinners)[-numTiedWinners:]]
        self.results[index,:clearIndices.shape[0]] = clearIndices
        self.results[index,clearIndices.shape[0]:] = tiedIndices
        
        self.timePerQuery.append(thisQueryTime)
        tEnd = time.time()
        self.resultProcTime += tEnd-tStart
        
        if self.benchmark:
            tStart = time.time()
            rawTiming = self.timingChannel.read(1)
            numMessages = rawTiming[0] & ((1<<8)-1)
            rawTiming.extend(self.timingChannel.read(numMessages-1))
            tEnd = time.time()
            self.timingRecTime += tEnd-tStart

            tStart = time.time()
            thisTimePerTimestep, thisNumSteps = self._interpretTiming(rawTiming)
            self.timePerTimestep.append(thisTimePerTimestep)
            self.numSteps.append(thisNumSteps)
            tEnd = time.time()
            self.timingProcTime += tEnd-tStart
        
    def test(self, queries, rawData, encodingFunction=None):
        """
        Sends queries to the Loihi system and receives the results.
        
        :param ndarray queries: 2D array of query samples, one per row
        :param ndarray rawData: 2D array of dataset samples, one per row
        :param function encodingFunction: Optional function to encode queries
        
        :returns: results, timePerQuery, numSteps, timePerTimestep
        :rtype: tuple
        """
        
        self.rawData  = rawData.copy()
        self.rawData -= np.expand_dims(self.rawData.mean(axis=1), axis=1)
        self.rawData /= np.expand_dims(np.linalg.norm(self.rawData, axis=1), axis=1) + np.finfo(float).eps
        
        self.encodingFunction = encodingFunction
        if self.encodingFunction is None:
            assert queries.shape[1] == self.inputDimension, "input must be of shape (n,{})".format(self.inputDimension)
        numTestIndices = queries.shape[0]
        runSteps = numTestIndices*self.queryDuration*2
        self.runTime += runSteps
        self.board.run(runSteps, aSync=True)
        self.spikeArray = np.ones((self.tokensPerPacket,), dtype=int)* ((63<<10) | 1022)
        self.results = np.zeros((numTestIndices,self.k), dtype=int)
        self.timePerTimestep = []
        self.timePerQuery = []
        self.numSteps = []
        
        self.pipelining = min(self.pipelining, numTestIndices)
        
        for ii in range(self.pipelining):
            #print("sending query {}".format(ii))
            self._sendQuery(queries[ii,:])
            
        for ii in range(self.pipelining,numTestIndices):
            self._getResult(queries[ii-self.pipelining,:], ii-self.pipelining)
            #print("sending query {}".format(ii))
            self._sendQuery(queries[ii,:])
            
        for ii in range(self.pipelining):
            self._getResult(queries[numTestIndices+ii-self.pipelining,:], numTestIndices+ii-self.pipelining)
        
        return self.results, self.timePerQuery, self.numSteps, self.timePerTimestep
    
    def finish(self):
        """
        Advances Loihi to the end of the run and disconnects
        """
        self.spikeArray[:] = ((63<<10)|1022)
        rawSpikeArray = self.spikeArray[::2] | (self.spikeArray[1::2]<<16)
        rawSpikeArray[rawSpikeArray>=(1<<31)] -= (1<<32)
        self.spikeChannel.write(self.tokensPerPacket//32, rawSpikeArray)
        self.board.finishRun()
        self.board.disconnect()
        
    def computeGroundTruth(self, datasetIn, queriesIn, k=None):
        """
        Finds the top K dataset samples for each query sample based on 
        cosine similarity.
        
        :param ndarray dataset: 2D array of dataset samples, one per row
        :param ndarray queries: 2D array of query samples, one per row
        :param int k: The number of top matches to return
        
        :returns: Array of top matches, one row per query
        :rtype: ndarray
        """
        
        if k is None:
            k=self.k
        
        dataset = datasetIn.copy()
        queries = queriesIn.copy()
        
        # normalize dataset
        dataset -= np.expand_dims(dataset.mean(axis=1), axis=1)
        dataset /= np.expand_dims(np.linalg.norm(dataset, axis=1), axis=1) + np.finfo(float).eps
        
        # normalize query(ies)
        if len(queries.shape)>1:
            queries -= np.expand_dims(queries.mean(axis=1), axis=1)
            queries /= np.expand_dims(np.linalg.norm(queries, axis=1), axis=1) + np.finfo(float).eps
        else:
            queries -= queries.mean()
            queries /= np.linalg.norm(queries) + np.finfo(float).eps
        
        result = np.argpartition(np.dot(queries, dataset.T), -k)[:,-k:]
        
        return result

    def _encodeSpikes(self, query):
        """
        Encodes a query as spike times. 
        If an encoding function is defined, the query will first be passed 
        to the encoding function before being converted to spike times.
        """
        
        # use encoding function if present
        if self.encodingFunction is not None:
            query = self.encodingFunction(query)
        else:
            query -= np.mean(query)
        
        # split positive and negative components
        query = np.concatenate((query, -query))
        
        # scale and sort to extract largest components
        input_spike_times = -(query*self.queryDuration/query.max()).astype(int)
        top = np.argpartition(input_spike_times, self.spikesPerQuery)[:self.spikesPerQuery]
        order = top[np.argsort(input_spike_times[top])]
        
        return input_spike_times[order], order
        
