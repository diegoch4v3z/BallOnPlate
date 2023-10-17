###############################################################
# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2019-2021 Intel Corporation.

# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.

# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.
###############################################################

import copy
import math

import numpy as np
from matplotlib import pyplot as plt


class ResourceEstimatorDist:
    """Estimates resource requirements of LCA network when distributing
    network equally across cores and chips."""

    def __init__(self, numWgtBits,
                 maxNumNeuronGroupsPerCore=16,
                 maxNumNeuronsPerCore=1024,
                 maxNumSynMemWordsPerCore=2**14,
                 maxNumAxonCfgWordsPerCore=2**12,
                 useMultiChip=True,
                 popMsgSize=32):

        self.maxNumNeuronsPerCore = maxNumNeuronsPerCore
        self.maxNumNeuronGroupsPerCore = maxNumNeuronGroupsPerCore
        self.maxNumSynMemWordsPerCore = maxNumSynMemWordsPerCore
        self.maxNumAxonCfgWordsPerCore = maxNumAxonCfgWordsPerCore

        self.numWgtBits = numWgtBits
        self._numSynPerSynEntry = 60
        self._numBitsPerSynMemWord = 64

        if popMsgSize == 16:
            self._axonCfgSpacing = 1
        elif popMsgSize == 32:
            self._axonCfgSpacing = 2
        else:
            raise ValueError('popMsgSize must be 16 or 32.')
        if useMultiChip:
            self._axonCfgSpacing += 1

        self.numNeuronsPerSlice = None
        self.numNeuronsPerCore = None
        self.numSynMemWordsPerCore = None
        self.numAxonCfgWordsPerCore = None
        self.numSlices = None
        self.numCores = None

    @staticmethod
    def _validateImgPatchStrideSize(imgSize, patchSize, strideSize):
        assert (strideSize == 0 and imgSize == patchSize) \
            or ((imgSize-patchSize) % strideSize == 0)
        assert strideSize == 0 or (patchSize % strideSize == 0)

    @staticmethod
    def _computeNumPatches1D(imgSize, patchSize, strideSize):
        re = ResourceEstimator
        re._validateImgPatchStrideSize(imgSize, patchSize, strideSize)
        if strideSize == 0:
            numPatches = 1
        else:
            numPatches = int((imgSize - patchSize) / strideSize + 1)

        return numPatches

    @staticmethod
    def _computeNumNeuronGroups(imgSizeX, patchSizeX, strideSizeX,
                                imgSizeY, patchSizeY, strideSizeY):

        re = ResourceEstimator
        nX = re._computeNumPatches1D(imgSizeX, patchSizeX, strideSizeX)
        nY = re._computeNumPatches1D(imgSizeY, patchSizeY, strideSizeY)

        return int(nX * nY)

    @staticmethod
    def _computeNumSynGroups1D(imgSize, patchSize, strideSize):
        re = ResourceEstimator
        re._validateImgPatchStrideSize(imgSize, patchSize, strideSize)
        if strideSize == 0:
            numGroups = 0
        else:
            maxNumGroups = re._computeNumPatches1D(imgSize, patchSize,
                                                   strideSize)
            numGroups = (patchSize/strideSize - 1) * 2
            numGroups = int(min(maxNumGroups-1, numGroups))

        return numGroups

    @staticmethod
    def _computeNumSynGroups(imgSizeX, patchSizeX, strideSizeX,
                             imgSizeY, patchSizeY, strideSizeY):

        re = ResourceEstimator
        nX = re._computeNumSynGroups1D(imgSizeX, patchSizeX, strideSizeX)
        nY = re._computeNumSynGroups1D(imgSizeY, patchSizeY, strideSizeY)

        return nX * nY + nX + nY + 1

    def _computeNumNeuronsPerSlice(self, maxNumNeuronsPerSlice):
        self.numNeuronsPerSlice = int(math.floor(
            self.maxNumNeuronsPerCore / self.numNeuronGroupsPerCore))
        self.numNeuronsPerSlice = min(self.numNeuronsPerGroup,
                                      self.numNeuronsPerSlice,
                                      maxNumNeuronsPerSlice)

    def compute(self, numNeuronsPerCore, numCoresPerChip, numChips,
                imgSize, patchSize, strideSize):

        self.computeXY(numNeuronsPerCore, numCoresPerChip, numChips,
                       imgSize, patchSize, strideSize,
                       imgSize, patchSize, strideSize)

    def computeXY(self, numNeuronsPerCore, numCoresPerChip, numChips,
                  imgSizeX, patchSizeX, strideSizeX,
                  imgSizeY, patchSizeY, strideSizeY):

        self.numNeuronsPerCore = numNeuronsPerCore
        raise AssertionError("Check if numNeuronsPerCore below is wrong.")
        self.numCoresPerChip = numNeuronsPerCore
        self.numChips = numChips

        self.numNeuronsPerChip = numNeuronsPerCore * numCoresPerChip
        self.numNeurons = self.numNeuronsPerChip * numChips
        self.numNeuronGroups = self._computeNumNeuronGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY)
        self.dictSize = self.numNeurons/(2*self.numNeuronGroups)
        self.numNeuronsPerGroup = 2*self.dictSize

        self.numSynGroupsPerCore = self._computeNumSynGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY)
        self.numNeuronGroupsPerCore = min(self.maxNumNeuronGroupsPerCore,
                                          self.numNeuronGroups)
        self._computeNumNeuronsPerSlice(1024)
        while True:
             # Reduce max. number of neurons per slice if number of synaptic
            # resources are exceeded
            self._computeNumSynMemWordsPerCore()
            if self.numSynMemWordsPerCore > self.maxNumSynMemWordsPerCore:
                self.numNeuronsPerSlice -= 1
                assert self.numNeuronsPerSlice > 0, \
                    'numNeuronsPerSlice cannot be zero. Network is likely too ' \
                    'big for numNeuronsPerGroup=%d and requires too many ' \
                    'synMemResources.' % (numNeuronsPerGroup)

            self._computeNumSlices()

            # Reduce number of neuron groups per core if number of axonal
            # resources are exceeded which is the only way to reduce number
            # of axonCfg entries.
            self._computeNumAxonCfgWordsPerCore()
            if self.numAxonCfgWordsPerCore > self.maxNumAxonMemWordsPerCore:
                self.numNeuronGroupsPerCore -= 1
                assert self.numNeuronGroupsPerCore > 0, \
                    'numNeuronGroupsPerCore cannot be zero. Network is likely ' \
                    'too big for numNeuronsPerGroup=%d and requires too many ' \
                    'axonCfgResources.' % (numNeuronsPerGroup)
                self._computeNumNeuronsPerSlice(self.numNeuronsPerSlice)

            if self.numSynMemWordsPerCore <= self.maxNumSynMemWordsPerCore \
                    and self.numAxonCfgWordsPerCore <= self.maxNumAxonMemWordsPerCore:
                break

    def _computeNumSynMemWordsPerCore(self):
        ignoreDummySynMemInsertion = False
        if ignoreDummySynMemInsertion:
            totalNumWgtsBits = self.numNeuronsPerSlice * self.numWgtBits
            numSynEntries = math.ceil(
                self.numNeuronsPerSlice / self._numSynPerSynEntry)
            totalNumPrefixBits = numSynEntries * 21
            totalNumBits = totalNumPrefixBits + totalNumWgtsBits
            numSynMemWordsPerAxon = math.ceil(
                totalNumBits / self._numBitsPerSynMemWord)
        else:
            # If synapses of an axon end on a full synMem word, then the
            # synapseCompiler will insert an empty synMemWord
            numPrefixBitsFull = 4 + 10
            numPrefixBitsPartial = numPrefixBitsFull + 6
            numSynEntriesFull = math.floor(self.numNeuronsPerSlice /
                                           self._numSynPerSynEntry)
            numSynPartial = self.numNeuronsPerSlice % self._numSynPerSynEntry
            totalNumBits = numSynEntriesFull \
                * (numPrefixBitsFull +
                   self._numSynPerSynEntry * self.numWgtBits) \
                + numPrefixBitsPartial \
                + numSynPartial * self.numWgtBits
            numSynMemWordsPerAxon = math.ceil(
                totalNumBits / self._numBitsPerSynMemWord)
            if totalNumBits % self._numBitsPerSynMemWord == 0:
                numSynMemWordsPerAxon += 1

        self.numSynMemWordsPerCore = numSynMemWordsPerAxon * \
            self.numNeuronsPerGroup * \
            self.numSynGroupsPerCore

    def _computeNumSlices(self):
        self.numSlices = int(math.ceil(
            self.numNeuronsPerGroup/self.numNeuronsPerSlice))

    def _computeNumAxonCfgWordsPerCore(self):
        self.numAxonCfgWordsPerCore = self.numNeuronGroupsPerCore \
            * self.numSlices \
            * self.numSynGroupsPerCore \
            * self._axonCfgSpacing

    def computeOld(self, numNeuronsPerGroup,
                   imgSize, patchSize, strideSize):
        """Same as computeXY but assumes imgSizeX=imgSizeY, \
        patchSizeX=patchSizeY, strideSizeX=strideSizeY.
        """

        self.computeXY(numNeuronsPerGroup,
                       imgSize, patchSize, strideSize,
                       imgSize, patchSize, strideSize)

    def computeXYOld(self, numNeuronsPerGroup,
                     imgSizeX, patchSizeX, strideSizeX,
                     imgSizeY, patchSizeY, strideSizeY):
        """Estimates maximum resource requirements.
        The first step is to compute the maximum number of neuronsPerSlice.
        From this the number of synMemWordsPerCore is computed. If the
        maximum permissible number of words is exceeded,
        the numNeuronsPerSlice is decremented until the synaptic resource
        requirements are satisfied.
        Similarly the number of axonCfgWordsPerCore is computed, which will
        increase with decreasing numNeuronsPerSlice and thus increasing
        numSlices. If the maximum permissible number is exceeded,
        the numNeuronGroupsPerCore is decremented and the search starts all
        over.
        If all requirements are satisfied, the search ends and the final
        number of numNeuronsPerCore and numCores is computed.

        :param int numNeuronsPerGroup: The number of neurons in a neuron \
        group used to represent a patch. This is typically twice the \
        dictionary size as we are assuming that we need two neurons to \
        represent positive and negative dictionary elements.
        :param int imgSize: Size of image. Must be a multiple of strideSize \
        plus patchSize.
        :param int patchSize: Size of a patch. Must be a multiple of strideSize.
        :param int strideSize: Size of a stride.
        """

        self.numNeuronsPerGroup = numNeuronsPerGroup
        self.numNeuronGroups = self._computeNumNeuronGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY)
        self.numSynGroupsPerCore = self._computeNumSynGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY)
        self.numNeuronGroupsPerCore = min(self.maxNumNeuronGroupsPerCore,
                                          self.numNeuronGroups)
        self._computeNumNeuronsPerSlice(1024)
        while True:
            # Reduce max. number of neurons per slice if number of synaptic
            # resources are exceeded
            self._computeNumSynMemWordsPerCore()
            if self.numSynMemWordsPerCore > self.maxNumSynMemWordsPerCore:
                self.numNeuronsPerSlice -= 1
                assert self.numNeuronsPerSlice > 0, \
                    'numNeuronsPerSlice cannot be zero. Network is likely too ' \
                    'big for numNeuronsPerGroup=%d and requires too many ' \
                    'synMemResources.' % (numNeuronsPerGroup)

            self._computeNumSlices()

            # Reduce number of neuron groups per core if number of axonal
            # resources are exceeded which is the only way to reduce number
            # of axonCfg entries.
            self._computeNumAxonCfgWordsPerCore()
            if self.numAxonCfgWordsPerCore > self.maxNumAxonCfgWordsPerCore:
                self.numNeuronGroupsPerCore -= 1
                assert self.numNeuronGroupsPerCore > 0, \
                    'numNeuronGroupsPerCore cannot be zero. Network is likely ' \
                    'too big for numNeuronsPerGroup=%d and requires too many ' \
                    'axonCfgResources.' % (numNeuronsPerGroup)
                self._computeNumNeuronsPerSlice(self.numNeuronsPerSlice)

            if self.numSynMemWordsPerCore <= self.maxNumSynMemWordsPerCore \
                    and self.numAxonCfgWordsPerCore <= self.maxNumAxonCfgWordsPerCore:
                break

        self.numNeuronsPerCore = self.numNeuronsPerSlice * \
            self.numNeuronGroupsPerCore

        self.numCores = int(math.ceil(self.numNeuronGroups
                                      / self.numNeuronGroupsPerCore)
                            * self.numSlices)

    def _sweepParam(self, cmpFct, sweepParams,
                    dictSize, imgSize, patchSize, strideSize,
                    showSweep, sweepParamName):

        numDataPoints = len(sweepParams)
        numNeuronsPerSlice = np.zeros(numDataPoints, int)
        numNeuronsPerCore = np.zeros(numDataPoints, int)
        numSynMemWordsPerCore = np.zeros(numDataPoints, int)
        numAxonCfgWordsPerCore = np.zeros(numDataPoints, int)
        numSlices = np.zeros(numDataPoints, int)
        numCores = np.zeros(numDataPoints, int)
        dummyRR = copy.deepcopy(self)
        for i, n in enumerate(sweepParams):
            cmpFct(dummyRR, n, dictSize, imgSize, patchSize, strideSize)
            numNeuronsPerSlice[i] = dummyRR.numNeuronsPerSlice
            numNeuronsPerCore[i] = dummyRR.numNeuronsPerCore
            numSynMemWordsPerCore[i] = dummyRR.numSynMemWordsPerCore
            numAxonCfgWordsPerCore[i] = dummyRR.numAxonCfgWordsPerCore
            numSlices[i] = dummyRR.numSlices
            numCores[i] = dummyRR.numCores

        if showSweep:
            self._showSweep(sweepParams, sweepParamName,
                            numNeuronsPerCore, numNeuronsPerSlice,
                            numSynMemWordsPerCore, numAxonCfgWordsPerCore,
                            numSlices, numCores)

        return numNeuronsPerCore, numNeuronsPerSlice, numSynMemWordsPerCore, \
            numAxonCfgWordsPerCore, numSlices, numCores

    def _showSweep(self, sweepParams, sweepParamName,
                   numNeuronsPerCore, numNeuronsPerSlice, numSynMemWordsPerCore,
                   numAxonCfgWordsPerCore, numSlices, numCores):

        plt.subplot(2, 3, 1)
        plt.plot(sweepParams, numNeuronsPerSlice)
        plt.xlabel(sweepParamName)
        plt.ylabel('numNeuronsPerSlice')

        plt.subplot(2, 3, 2)
        plt.plot(sweepParams, numNeuronsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numNeuronsPerCore')

        plt.subplot(2, 3, 3)
        plt.plot(sweepParams, numSynMemWordsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numSynMemWordsPerCore')

        plt.subplot(2, 3, 4)
        plt.plot(sweepParams, numAxonCfgWordsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numAxonCfgWordsPerCore')

        plt.subplot(2, 3, 5)
        plt.plot(sweepParams, numSlices)
        plt.xlabel(sweepParamName)
        plt.ylabel('numSlices')

        plt.subplot(2, 3, 6)
        plt.plot(sweepParams, numCores)
        plt.xlabel(sweepParamName)
        plt.ylabel('numCores')
        xlim = plt.xlim()
        ylim = plt.ylim()
        plt.hlines(128, *xlim, linestyle='--', color='r')
        plt.hlines(128*4, *xlim, linestyle='--', color='g')
        plt.hlines(128*32, *xlim, linestyle='--', color='b')
        plt.ylim(ylim)

        plt.tight_layout()

    def sweepDictSize(self, dictSizes, imgSize, patchSize, strideSize,
                      showSweep=False):
        """Sweeps dictionary size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(sp*2, ims, ps, ss)

        return self._sweepParam(cmpFct, dictSizes,
                                None, imgSize, patchSize, strideSize,
                                showSweep, 'dictSize')

    def sweepImgSize(self, dictSize, imgSizes, patchSize, strideSize,
                     showSweep=False):
        """Sweeps image size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, sp, ps, ss)

        return self._sweepParam(cmpFct, imgSizes,
                                dictSize, None, patchSize, strideSize,
                                showSweep, 'imgSize')

    def sweepPatchSize(self, dictSize, imgSize, patchSizes, strideSize,
                       showSweep=False):
        """Sweeps patch size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, ims, sp, ss)

        return self._sweepParam(cmpFct, patchSizes,
                                dictSize, imgSize, None, strideSize,
                                showSweep, 'patchSize')

    def sweepStrideSize(self, dictSize, imgSize, patchSize, strideSizes,
                        showSweep=False):
        """Sweeps stride size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, ims, ps, sp)

        return self._sweepParam(cmpFct, strideSizes,
                                dictSize, imgSize, patchSize, None,
                                showSweep, 'strideSize')


class ResourceEstimator:
    """Computes and contains all parameters that describe the resource
    requirements of an LCA network."""

    def __init__(self, numWgtBits,
                 maxNumNeuronGroupsPerCore=16,
                 maxNumNeuronsPerCore=1024,
                 maxNumSynMemWordsPerCore=2**14,
                 maxNumAxonCfgWordsPerCore=2**12,
                 useMultiChip=True,
                 popMsgSize=32,
                 maxConnectionDistance=None):

        self.maxNumNeuronsPerCore = maxNumNeuronsPerCore
        self.maxNumNeuronGroupsPerCore = maxNumNeuronGroupsPerCore
        self.maxNumSynMemWordsPerCore = maxNumSynMemWordsPerCore
        self.maxNumAxonCfgWordsPerCore = maxNumAxonCfgWordsPerCore

        self.numWgtBits = numWgtBits
        self._numSynPerSynEntry = 60
        self._numBitsPerSynMemWord = 64

        if popMsgSize == 16:
            self._axonCfgSpacing = 1
        elif popMsgSize == 32:
            self._axonCfgSpacing = 2
        else:
            raise ValueError('popMsgSize must be 16 or 32.')
        if useMultiChip:
            self._axonCfgSpacing += 1
           
        if maxConnectionDistance is None:
            maxConnectionDistance = 1e10
        self.maxConnectionDistance = maxConnectionDistance

        self.numNeuronsPerSlice = None
        self.numNeuronsPerCore = None
        self.numSynMemWordsPerCore = None
        self.numAxonCfgWordsPerCore = None
        self.numSlices = None
        self.numCores = None

    def _computeNumNeuronsPerSlice(self, maxNumNeuronsPerSlice):
        self.numNeuronsPerSlice = int(math.floor(
            self.maxNumNeuronsPerCore / self.numNeuronGroupsPerCore))
        self.numNeuronsPerSlice = min(self.numNeuronsPerGroup,
                                      self.numNeuronsPerSlice,
                                      maxNumNeuronsPerSlice)

    def _computeNumSynMemWordsPerCore(self):
        ignoreDummySynMemInsertion = False
        if ignoreDummySynMemInsertion:
            totalNumWgtsBits = self.numNeuronsPerSlice * self.numWgtBits
            numSynEntries = math.ceil(
                self.numNeuronsPerSlice / self._numSynPerSynEntry)
            totalNumPrefixBits = numSynEntries * 21
            totalNumBits = totalNumPrefixBits + totalNumWgtsBits
            numSynMemWordsPerAxon = math.ceil(
                totalNumBits / self._numBitsPerSynMemWord)
        else:
            # If synapses of an axon end on a full synMem word, then the
            # synapseCompiler will insert an empty synMemWord
            numPrefixBitsFull = 4 + 10
            numPrefixBitsPartial = numPrefixBitsFull + 6
            numSynEntriesFull = math.floor(self.numNeuronsPerSlice /
                                           self._numSynPerSynEntry)
            numSynPartial = self.numNeuronsPerSlice % self._numSynPerSynEntry
            totalNumBits = numSynEntriesFull \
                * (numPrefixBitsFull +
                   self._numSynPerSynEntry * self.numWgtBits) \
                + numPrefixBitsPartial \
                + numSynPartial * self.numWgtBits
            numSynMemWordsPerAxon = math.ceil(
                totalNumBits / self._numBitsPerSynMemWord)
            if totalNumBits % self._numBitsPerSynMemWord == 0:
                numSynMemWordsPerAxon += 1

        self.numSynMemWordsPerCore = numSynMemWordsPerAxon * \
            self.numNeuronsPerGroup * \
            self.numSynGroupsPerCore

    def _computeNumSlices(self):
        self.numSlices = int(math.ceil(
            self.numNeuronsPerGroup/self.numNeuronsPerSlice))

    def _computeNumAxonCfgWordsPerCore(self):
        self.numAxonCfgWordsPerCore = self.numNeuronGroupsPerCore \
            * self.numSlices \
            * self.numSynGroupsPerCore \
            * self._axonCfgSpacing

    @staticmethod
    def _validateImgPatchStrideSize(imgSize, patchSize, strideSize):
        assert (strideSize == 0 and imgSize == patchSize) \
            or ((imgSize-patchSize) % strideSize == 0)
        assert strideSize == 0 or (patchSize % strideSize == 0)

    @staticmethod
    def _computeNumPatches1D(imgSize, patchSize, strideSize):
        re = ResourceEstimator
        re._validateImgPatchStrideSize(imgSize, patchSize, strideSize)
        if strideSize == 0:
            numPatches = 1
        else:
            numPatches = int((imgSize - patchSize) / strideSize + 1)

        return numPatches

    @staticmethod
    def _computeNumNeuronGroups(imgSizeX, patchSizeX, strideSizeX,
                                imgSizeY, patchSizeY, strideSizeY):

        re = ResourceEstimator
        nX = re._computeNumPatches1D(imgSizeX, patchSizeX, strideSizeX)
        nY = re._computeNumPatches1D(imgSizeY, patchSizeY, strideSizeY)

        return int(nX * nY)

    @staticmethod
    def _computeNumSynGroups1D(imgSize, patchSize, strideSize,
                               maxConnectionDistance):
        re = ResourceEstimator
        re._validateImgPatchStrideSize(imgSize, patchSize, strideSize)
        if strideSize == 0:
            numGroups = 0
        else:
            maxNumGroups = re._computeNumPatches1D(imgSize, patchSize,
                                                   strideSize)
            connectionDistance = patchSize/strideSize - 1
            numGroups = min(connectionDistance, maxConnectionDistance) * 2
            numGroups = int(min(maxNumGroups-1, numGroups))

        return numGroups

    @staticmethod
    def _computeNumSynGroups(imgSizeX, patchSizeX, strideSizeX,
                             imgSizeY, patchSizeY, strideSizeY,
                             maxConnectionDistance):

        re = ResourceEstimator
        nX = re._computeNumSynGroups1D(imgSizeX, patchSizeX, strideSizeX, maxConnectionDistance)
        nY = re._computeNumSynGroups1D(imgSizeY, patchSizeY, strideSizeY, maxConnectionDistance)

        return nX * nY + nX + nY + 1

    def compute(self, numNeuronsPerGroup,
                imgSize, patchSize, strideSize):
        """Same as computeXY but assumes imgSizeX=imgSizeY, \
        patchSizeX=patchSizeY, strideSizeX=strideSizeY.
        """

        self.computeXY(numNeuronsPerGroup,
                       imgSize, patchSize, strideSize,
                       imgSize, patchSize, strideSize)

    def computeXY(self, numNeuronsPerGroup,
                  imgSizeX, patchSizeX, strideSizeX,
                  imgSizeY, patchSizeY, strideSizeY):
        """Estimates maximum resource requirements.
        The first step is to compute the maximum number of neuronsPerSlice.
        From this the number of synMemWordsPerCore is computed. If the
        maximum permissible number of words is exceeded,
        the numNeuronsPerSlice is decremented until the synaptic resource
        requirements are satisfied.
        Similarly the number of axonCfgWordsPerCore is computed, which will
        increase with decreasing numNeuronsPerSlice and thus increasing
        numSlices. If the maximum permissible number is exceeded,
        the numNeuronGroupsPerCore is decremented and the search starts all
        over.
        If all requirements are satisfied, the search ends and the final
        number of numNeuronsPerCore and numCores is computed.

        :param int numNeuronsPerGroup: The number of neurons in a neuron \
        group used to represent a patch. This is typically twice the \
        dictionary size as we are assuming that we need two neurons to \
        represent positive and negative dictionary elements.
        :param int imgSize: Size of image. Must be a multiple of strideSize \
        plus patchSize.
        :param int patchSize: Size of a patch. Must be a multiple of strideSize.
        :param int strideSize: Size of a stride.
        """

        self.numNeuronsPerGroup = numNeuronsPerGroup
        self.numNeuronGroups = self._computeNumNeuronGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY)

        self.numSynGroupsPerCore = self._computeNumSynGroups(
            imgSizeX, patchSizeX, strideSizeX,
            imgSizeY, patchSizeY, strideSizeY,
            self.maxConnectionDistance)
        self.numNeuronGroupsPerCore = min(self.maxNumNeuronGroupsPerCore,
                                          self.numNeuronGroups)
        self._computeNumNeuronsPerSlice(1024)
        while True:
            # Reduce max. number of neurons per slice if number of synaptic
            # resources are exceeded
            self._computeNumSynMemWordsPerCore()
            if self.numSynMemWordsPerCore > self.maxNumSynMemWordsPerCore:
                self.numNeuronsPerSlice -= 1
                assert self.numNeuronsPerSlice > 0, \
                    'numNeuronsPerSlice cannot be zero. Network is likely too ' \
                    'big for numNeuronsPerGroup=%d and requires too many ' \
                    'synMemResources.' % (numNeuronsPerGroup)

            self._computeNumSlices()

            # Reduce number of neuron groups per core if number of axonal
            # resources are exceeded which is the only way to reduce number
            # of axonCfg entries.
            self._computeNumAxonCfgWordsPerCore()
            if self.numAxonCfgWordsPerCore > self.maxNumAxonCfgWordsPerCore:
                self.numNeuronGroupsPerCore -= 1
                assert self.numNeuronGroupsPerCore > 0, \
                    'numNeuronGroupsPerCore cannot be zero. Network is likely ' \
                    'too big for numNeuronsPerGroup=%d and requires too many ' \
                    'axonCfgResources.' % (numNeuronsPerGroup)
                self._computeNumNeuronsPerSlice(self.numNeuronsPerSlice)

            if self.numSynMemWordsPerCore <= self.maxNumSynMemWordsPerCore \
                    and self.numAxonCfgWordsPerCore <= self.maxNumAxonCfgWordsPerCore:
                break

        self.numNeuronsPerCore = self.numNeuronsPerSlice * \
            self.numNeuronGroupsPerCore

        self.numCores = int(math.ceil(self.numNeuronGroups
                                      / self.numNeuronGroupsPerCore)
                            * self.numSlices)

    def _sweepParam(self, cmpFct, sweepParams,
                    dictSize, imgSize, patchSize, strideSize,
                    showSweep, sweepParamName, numCoresPerChip=128):

        numDataPoints = len(sweepParams)
        numNeuronsPerSlice = np.zeros(numDataPoints, int)
        numNeuronsPerCore = np.zeros(numDataPoints, int)
        numSynMemWordsPerCore = np.zeros(numDataPoints, int)
        numAxonCfgWordsPerCore = np.zeros(numDataPoints, int)
        numSlices = np.zeros(numDataPoints, int)
        numCores = np.zeros(numDataPoints, int)
        dummyRR = copy.deepcopy(self)
        for i, n in enumerate(sweepParams):
            cmpFct(dummyRR, n, dictSize, imgSize, patchSize, strideSize)
            numNeuronsPerSlice[i] = dummyRR.numNeuronsPerSlice
            numNeuronsPerCore[i] = dummyRR.numNeuronsPerCore
            numSynMemWordsPerCore[i] = dummyRR.numSynMemWordsPerCore
            numAxonCfgWordsPerCore[i] = dummyRR.numAxonCfgWordsPerCore
            numSlices[i] = dummyRR.numSlices
            numCores[i] = dummyRR.numCores

        if showSweep:
            self._showSweep(sweepParams, sweepParamName,
                            numNeuronsPerCore, numNeuronsPerSlice,
                            numSynMemWordsPerCore, numAxonCfgWordsPerCore,
                            numSlices, numCores, numCoresPerChip)

        return numNeuronsPerCore, numNeuronsPerSlice, numSynMemWordsPerCore, \
            numAxonCfgWordsPerCore, numSlices, numCores

    def _showSweep(self, sweepParams, sweepParamName,
                   numNeuronsPerCore, numNeuronsPerSlice, numSynMemWordsPerCore,
                   numAxonCfgWordsPerCore, numSlices, numCores,
                   numCoresPerChip):

        plt.subplot(2, 3, 1)
        plt.plot(sweepParams, numNeuronsPerSlice)
        plt.xlabel(sweepParamName)
        plt.ylabel('numNeuronsPerSlice')

        plt.subplot(2, 3, 2)
        plt.plot(sweepParams, numNeuronsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numNeuronsPerCore')

        plt.subplot(2, 3, 3)
        plt.plot(sweepParams, numSynMemWordsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numSynMemWordsPerCore')

        plt.subplot(2, 3, 4)
        plt.plot(sweepParams, numAxonCfgWordsPerCore)
        plt.xlabel(sweepParamName)
        plt.ylabel('numAxonCfgWordsPerCore')

        plt.subplot(2, 3, 5)
        plt.plot(sweepParams, numSlices)
        plt.xlabel(sweepParamName)
        plt.ylabel('numSlices')

        plt.subplot(2, 3, 6)
        plt.plot(sweepParams, numCores)
        plt.xlabel(sweepParamName)
        plt.ylabel('numCores')
        xlim = plt.xlim()
        ylim = plt.ylim()
        plt.hlines(numCoresPerChip, *xlim, linestyle='--', color='r')
        plt.hlines(numCoresPerChip*4, *xlim, linestyle='--', color='g')
        plt.hlines(numCoresPerChip*32, *xlim, linestyle='--', color='b')
        plt.ylim(ylim)

        plt.tight_layout()

    def sweepMaxNumNeurons(self, maxNumNeurons,
                           dictSize, imgSize, patchSize, strideSize,
                           showSweep=False, numCoresPerChip=128):
        """Sweeps maximum number of neurons per core."""

        def cmpFct(re, sp, ds, ims, ps, ss):
            re.maxNumNeuronsPerCore = sp
            re.compute(ds*2, ims, ps, ss)

        return self._sweepParam(cmpFct, maxNumNeurons,
                                dictSize, imgSize, patchSize, strideSize,
                                showSweep, 'maxNumNeuronsPerCore', numCoresPerChip)

    def sweepDictSize(self, dictSizes, imgSize, patchSize, strideSize,
                      showSweep=False):
        """Sweeps dictionary size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(sp*2, ims, ps, ss)

        return self._sweepParam(cmpFct, dictSizes,
                                None, imgSize, patchSize, strideSize,
                                showSweep, 'dictSize')

    def sweepImgSize(self, dictSize, imgSizes, patchSize, strideSize,
                     showSweep=False):
        """Sweeps image size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, sp, ps, ss)

        return self._sweepParam(cmpFct, imgSizes,
                                dictSize, None, patchSize, strideSize,
                                showSweep, 'imgSize')

    def sweepPatchSize(self, dictSize, imgSize, patchSizes, strideSize,
                       showSweep=False):
        """Sweeps patch size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, ims, sp, ss)

        return self._sweepParam(cmpFct, patchSizes,
                                dictSize, imgSize, None, strideSize,
                                showSweep, 'patchSize')

    def sweepStrideSize(self, dictSize, imgSize, patchSize, strideSizes,
                        showSweep=False):
        """Sweeps stride size."""

        def cmpFct(re, sp, ds, ims, ps, ss): return \
            re.compute(ds*2, ims, ps, sp)

        return self._sweepParam(cmpFct, strideSizes,
                                dictSize, imgSize, patchSize, None,
                                showSweep, 'strideSize')
