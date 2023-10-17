# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2018-2021 Intel Corporation.

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
from nxsdk_modules.slic.src.nxcore.slicnetparams import *
from nxsdk_modules.slic.src.nxcore.modelparams import ModelParams
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.monitor.probes import *
from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen


class SlicNet:
    def __init__(self,
                 trainLabelData,
                 trainImageData,
                 inferLabelData,
                 inferImageData,
                 numSegments=4,
                 bytesPerImage=0,
                 xImgSize=20,
                 yImgSize=20,
                 debug=False):
        """
        Creates SlicNet network using NxCore API
        :param trainLabelData: Training label data in the format (numImage, xPixelData, yPixelData)
        :param trainImageData: Training image data in the format (numImage, data)
        :param inferLabelData: Inference label data in the format (numImage, xPixelData, yPixelData)
        :param inferImageData: Inference image data in the format (numImage, data)
        :param numSegments: Number of segments a line is divided into,
                            by the linescanner. Each segment corresponds
                            to an axon.
        :param xImgSize: Size of input image in X direction, in pixels
        :param yImgSize: Size of input image in y direction, in pixels
        :param debug: To toggle debug
        """

        self.trainingPhase = 1
        self.modelParams = ModelParams()
        # Initializing The Model Parameters
        self.currentTime = 1
        self.numInferImages = 0
        self.numCorrect = 0
        self.bytesPerImage = bytesPerImage
        self.xImgSize = xImgSize
        self.yImgSize = yImgSize
        self.debug = debug
        self.trainLabelData = trainLabelData
        self.trainImageData = trainImageData
        self.inferImageDataSet = inferImageData
        self.inferLabelDataSet = inferLabelData
        self._processLabelsAndImages()

        self.numLines = 18 * \
            (max(self.xImgSize, self.yImgSize) +
             max(self.xImgSize, self.yImgSize)//2)-64
        self.numAxons = self.numLines * numSegments

        self.board = N2Board(0, 1, [1], [[self.numAxons*self.numCx]])
        self.n2Core = self.board.n2Chips[0].n2Cores[0]
        self.params = SlicParams()
        # Setting up the activity probes
        self.setupActivityProbe(tStart=self.modelParams.timestepPerImage
                                * self.modelParams.numTrainIterations
                                * self.numTrainImages + self.modelParams.imagePresentOffset + 1)

    def _configureCx(self):
        numCx = self.numCx
        for cxId in range(numCx):
            self.n2Core.somaState[cxId].configure(vth=self.params.thresholdV)
            self.n2Core.cxCfg[cxId].configure(cxProfile=0,
                                              vthProfile=0)
        for i in range(numCx//4+1):
            self.n2Core.cxMetaState[i].configure(somaOp1=3)

        self.n2Core.cxProfileCfg[0].configure(bapAction=3,
                                              bapSrc=1,
                                              refractDelay=self.params.refractDelayTs,
                                              decayV=self.params.decayV,
                                              decayU=self.params.decayU)

        self.n2Core.vthProfileCfg[0].dynamicCfg.configure(enableHomeostasis=1,
                                                          beta=0,
                                                          aMin=0,
                                                          aMax=127)

        self.n2Core.dendriteAccumCfg.configure(delayBits=6)
        self.n2Core.dendriteTimeState.configure(tepoch=self.params.trainEpoch)
        self.n2Core.dendriteSharedCfg.configure(negVmLimit=self.params.negvmLimit,
                                                posVmLimit=self.params.posvmLimit)

    def _configureConnectivity(self):
        numCx = self.numCx
        numAxons = self.numAxons
        # Configure synapseFormat
        self.n2Core.synapseFmt[1].configure(learningCfg=3,
                                            wgtBits=7,
                                            numSynapses=63,
                                            skipBits=0,
                                            idxBits=5,
                                            fanoutType=1)

        # Configure synapses
        for i in range(0, numAxons * numCx):
            self.n2Core.synapses[i].configure(synFmtId=1,
                                              CIdx=i % numCx,
                                              Wgt=0,
                                              Dly=0,
                                              Tag=0,
                                              LrnEn=1)
        synPtr = 0
        for i in range(0, numAxons*2, 2):
            self.n2Core.synapseMap[i].synapsePtr = synPtr
            self.n2Core.synapseMap[i].synapseLen = numCx
            self.n2Core.synapseMap[i].discreteMapEntry.configure()
            self.n2Core.synapseMap[i+1].singleTraceEntry.configure()
            synPtr += numCx
        tf = TraceCfgGen()
        somaDecay = tf.calcDecay(self.params.traceCfgTau)
        rndThreshold = tf.calcRndThresh(self.params.traceCfgTau)
        rthMultiplier = tf.calcExpTimeConst(self.params.traceCfgTau)

        # Initialize somaTraceCfg
        self.n2Core.somaTraceCfg[0].spikeLevelInt = 1
        self.n2Core.somaTraceCfg[0].spikeLevelFrac = 0
        self.n2Core.somaTraceCfg[0].rthMultiplier = rthMultiplier
        self.n2Core.somaTraceCfg[0].randomThreshold = rndThreshold
        self.n2Core.somaTraceCfg[0].decay0 = somaDecay[0]
        self.n2Core.somaTraceCfg[0].decay1 = somaDecay[1]
        self.n2Core.somaTraceCfg[0].decay2 = somaDecay[2]
        self.n2Core.somaTraceCfg[0].decay3 = somaDecay[3]
        self.n2Core.somaTraceCfg[0].decay4 = somaDecay[4]
        self.n2Core.somaTraceCfg[0].decay5 = somaDecay[5]

    def _configureLearning(self):
        numCx = self.numCx
        # Configure pre trace related learning parameters
        self.n2Core.stdpCfg.firstLearningIndex = 0

        self.n2Core.stdpPreProfileCfg[0].updateOnX1 = 1

        self.n2Core.stdpPreCfg[0].configure(spikeLevelFrac=100,
                                            spikeLevelInt=127,
                                            rthMultiplier=51,
                                            randomThreshold=11,
                                            decay0=116,
                                            decay1=105,
                                            decay2=86,
                                            decay3=58,
                                            decay4=26,
                                            decay5=5,
                                            numDecayExp=6,
                                            traceBits=7)

        self.n2Core.stdpProfileCfg[0].configure(
            decimateExp=2,
            numProducts=1,
            requireY=1,
            usesXepoch=1,
            uCodePtr=0)

        self.n2Core.stdpProfileCfg[1].configure(
            decimateExp=2,
            numProducts=2,
            requireY=1,
            usesXepoch=1,
            uCodePtr=1)

        self.n2Core.stdpUcodeMem[0].word = self.params.stdpUcodeMemEntryDepressiveOnly
        self.n2Core.stdpUcodeMem[1].word = self.params.stdpUcodeMemEntrySStdp
        self.n2Core.timeState.tepoch = self.params.trainEpoch

    def _configureMisc(self):
        numCx = self.numCx
        numAxons = self.numAxons
        self.n2Core.timeState.tEpoch = self.params.trainEpoch
        self.n2Core.numUpdates.numUpdates = numCx // 4 + 1
        self.n2Core.numUpdates.numStdp = numAxons

    def _createInitSnip(self):
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/initsnip.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "initsnip"
        guardName = None
        phase = "init"
        self.initProcess = self.board.createProcess("initProcess", cPath, includeDir,
                                                    funcName, guardName, phase)

    def _createLineScanSnip(self):
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/linespiking.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "run_spiking"
        guardName = "do_spiking"
        phase = "spiking"
        self.spikingProcess = self.board.createProcess("spikingProcess", cPath, includeDir,
                                                       funcName, guardName, phase)

    def _createRunMgmtSnip(self):
        # Creating another snip to do management whenever domgmt condition is true
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "run_mgmt"
        guardName = "do_run_mgmt"
        phase = "mgmt"
        self.runMgmtProcess = self.board.createProcess("runMgmt", cPath, includeDir,
                                                       funcName, guardName, phase)

    def _createChannels(self):
        # Create a channel named initChannel for sending the network parameters
        self.initChannel = self.board.createChannel(b'nxinit', "int", 30)
        # Connecting imageChannel from initProcess to SuperHost making it receive channel
        self.initChannel.connect(None, self.initProcess)

        # Create a channel named imageChannel for sending image data
        self.imageChannel = self.board.createChannel(b'nximage', "int",
                                                     self.numTrainImages * self.modelParams.bytesPerImage)
        # Connecting imageChannel from Superhost to spikingProcess
        self.imageChannel.connect(None, self.spikingProcess)

        # Create a channel named labelChannel for sending label data
        self.labelChannel = self.board.createChannel(b'nxlabel', "int",
                                                     self.numTrainImages * self.modelParams.bytesPerImage)
        # Connecting labelChannel from Superhost to spikingProcess
        self.labelChannel.connect(None, self.runMgmtProcess)

        self.board.start()

    def _sendNetworkConfiguration(self):
        # Writing all the model params to pass it to the snip
        self.initChannel.write(1, [self.numCx])
        self.initChannel.write(1, [self.numAxons])
        self.initChannel.write(1, [self.n2Core.id])
        self.initChannel.write(1, [self.modelParams.timestepPerImage])
        self.initChannel.write(1, [self.modelParams.numTrainIterations])
        self.initChannel.write(1, [self.modelParams.imagePresentOffset])
        self.initChannel.write(1, [self.numTrainImages])
        self.initChannel.write(1, [self.modelParams.bytesPerImage])
        self.initChannel.write(1, [self.modelParams.firstTimestep])
        self.initChannel.write(1, [self.modelParams.spikeInterval])

    def _createSnipsAndChannels(self):
        self._createInitSnip()
        self._createLineScanSnip()
        self._createRunMgmtSnip()
        self._createChannels()
        self._sendNetworkConfiguration()

    def _sendTrainingImageData(self, start, end):
        self.imageChannel.write(
            end-start+1, self.trainImageData[start:(end+1)])

    def _sendTrainingLabelData(self, start, end):
        self.labelChannel.write(
            end-start+1, self.trainLabelData[start:(end+1)])

    def _processLabelsAndImages(self):
        """
        Takes image training and label files, opens them and process them according to the phase
         it is in (training, inference).
        :param labels: Label file
        :param images: Training file
        """
        if self.trainingPhase == 1:
            # Training Phase
            if self.debug:
                print("Training Phase")
            self.numCx = len(np.unique(self.trainLabelData))
            self.numTrainImages = len(
                self.trainImageData)//(self.bytesPerImage)
            if self.numTrainImages > self.modelParams.maxNumImages:
                self.numTrainImages = self.modelParams.maxNumImages
            print("Training images : {}".format(self.numTrainImages))
        else:
            # Inference Phase
            if self.debug:
                print("Inference Phase")
            try:
                self.inferImageData = self.inferImageDataSet[self.numInferImages*self.bytesPerImage:(
                    self.numInferImages+1)*self.bytesPerImage]
                self.inferLabelData = self.inferLabelDataSet[self.numInferImages]
                if len(self.inferImageData) != self.bytesPerImage:
                    return 0
            except IndexError:
                print("Returning 0 : Out of Index")
                return 0
        return 1

    def _checkInferenceAccuracy(self, correctLabel=None):
        """
        Function to check if the image is inferred correctly.
        """
        if self.debug:
            print("Checking for Inference Accuracy at time :{}".format(
                self.currentTime))
        maxSpikes = 0
        for i in range(self.numCx):
            if max(self.probeActivity[i].data[-self.modelParams.timestepPerImage:]) >= maxSpikes:
                predictedLabel = i
                maxSpikes = max(
                    self.probeActivity[i].data[-self.modelParams.timestepPerImage:])
            if self.debug:
                print("Data is : ", maxSpikes)
        if maxSpikes == 0:
            print("FAIL: Expected label {} No Spikes! Object not recognized\n".format(
                correctLabel))
            predictedLabel = -1
        else:
            if(predictedLabel == correctLabel):
                print("PASS: Expected label={} Received label={} \n".format(
                    correctLabel,
                    predictedLabel))
                self.numCorrect += 1
            else:
                print("FAIL: Expected label={} Received label={} \n".format(
                    correctLabel,
                    predictedLabel))
        return predictedLabel

    def setupNetwork(self):
        self._configureCx()
        self._configureConnectivity()
        self._configureLearning()
        self._configureMisc()
        self._createSnipsAndChannels()

    def setupSpikeProbes(self, tStart, tReset):
        numCx = self.numCx
        customSpikeProbeCond = SpikeProbeCondition(tReset, tStart)
        mon = self.board.monitor
        cx = list(range(numCx))
        self.probeSpikes = mon.probe(
            self.n2Core.cxState, cx, 'spike', probeCondition=customSpikeProbeCond)

    def setupActivityProbe(self, dt=1, tStart=0):
        """
        Sets up the activity probe
        :param dt: Offset
        :param tStart: When to start recording the state
        :return:
        """
        numCx = self.numCx
        mon = self.board.monitor
        pc = IntervalProbeCondition(dt, tStart)
        cx = list(range(numCx))
        self.probeActivity = mon.probe(self.n2Core.somaState, cx, 'a', pc)

    def setupCxProbe(self):
        """
        Sets up the probe for compartment current and voltage state
        """
        numCx = self.numCx
        mon = self.board.monitor
        cx = list(range(numCx))
        self.probeCxU = mon.probe(self.n2Core.cxState, cx, 'u')
        self.probeCxV = mon.probe(self.n2Core.cxState, cx, 'v')

    def setupSynapseProbe(self):
        """
        Sets up synapse probe
        """
        numCx = self.numCx
        numAxons = self.numAxons
        mon = self.board.monitor
        cx = list(range(numAxons*numCx))
        self.probeSynapse = mon.probe(self.n2Core.synapses, cx, 'Wgt')

    def runTraining(self):
        """
        Runs training on the network
        """
        print("Num iterations is : ", self.modelParams.numTrainIterations)
        for i in range(self.modelParams.numTrainIterations):
            self._sendTrainingImageData(
                start=0, end=len(self.trainImageData)-1)
            self._sendTrainingLabelData(
                start=0, end=len(self.trainLabelData)-1)
            self.board.run(self.modelParams.imagePresentOffset
                           + (self.numTrainImages*self.modelParams.timestepPerImage-1))
            self.currentTime += self.modelParams.imagePresentOffset +\
                (self.numTrainImages*self.modelParams.timestepPerImage)
            print("Completed Iteration : ", i)
        print("Training Complete Now. Time for Inference")
        if self.debug:
            self.getWeightMatrix(file="/tmp/wgtMatrix.out")

    def runInference(self):
        """
        Runs inference on the network
        """
        self.runModelMgt(self.currentTime)
        while(self._processLabelsAndImages()):
            self.runInferenceOnImageDataStream(
                self.inferImageData, self.inferLabelData)
            print("Completed Inferring {} Image".format(self.numInferImages))
        print("Inference Complete : Accuracy is {}".format(
            self.numCorrect / self.numInferImages))
        return self.numCorrect / self.numInferImages

    def runInferenceOnImageDataStream(self, imageData, label):
        """
        Runs inference given the imageData and label
        :param imageData: image data
        :param label: actual label
        :return: predicted label
        """
        self.runModelMgt(self.currentTime)
        if len(imageData) != self.bytesPerImage:
            raise ValueError("Image of Wrong Size Provided")
        self.imageChannel.write(len(imageData), imageData)
        self.board.run(self.modelParams.timestepPerImage)
        self.currentTime += self.modelParams.timestepPerImage
        label = self._checkInferenceAccuracy(label)
        self.clearActivityCounters(self.numCx)
        self.numInferImages += 1
        return label

    def getInferenceAccuracy(self):
        """
        Returns the inference accuracy
        """
        if self.numInferImages > 0:
            return self.numCorrect / self.numInferImages
        else:
            return 0

    def clearActivityCounters(self, numCx):
        """
        Function to clear the activity counters.
        :param numCx: Number of Compartments
        """
        for i in range(numCx):
            self.n2Core.somaState[i].configure(a=0)

    def runModelMgt(self, currentTime):
        """
        Function depending on the currentTime of execution does Management.
        :param currentTime: Current time of execution.
        """
        if self.trainingPhase:
            if currentTime >= self.modelParams.timestepPerImage * self.modelParams.numTrainIterations \
                    * self.numTrainImages + self.modelParams.imagePresentOffset:
                print("Training Complete : Time to disable the learning")
                self.trainingPhase = 0

    # Debug function to print activity probes for timestepPerImage timesteps.
    def printActivityProbes(self):
        for i in range(self.numCx):
            print("Activity Probe : {} is {}".format(i,
                                                     self.probeActivity[i].timeSeries.data[-self.modelParams.timestepPerImage:]))

    # Debug function to print spike probes for timestepPerImage timesteps.
    # Spike Probes should have been set before use.
    def printSpikeProbes(self):
        for i in range(self.numCx):
            print("Spike Probe : {} is {}".format(i,
                                                  self.probeSpikes[i].timeSeries.data[-self.modelParams.timestepPerImage:]))

    # Debug function to print Cx - u state probes for timestepPerImage timesteps.
    # Cx - u Probes should have been set before use.
    def printCxUProbes(self):
        for i in range(self.numCx):
            print("Cx U : {} is {}".format(i,
                                           self.probeCxU[i].timeSeries.data[-self.modelParams.timestepPerImage:]))

    # Debug function to print Cx - v state probes for timestepPerImage timesteps.
    # Cx - v Probes should have been set before use.
    def printCxVProbes(self):
        for i in range(self.numCx):
            print("Cx V : {} is {}".format(i,
                                           self.probeCxV[i].timeSeries.data[-self.modelParams.timestepPerImage:]))

    # Debug function to print weight matrix
    def printWeightMatrix(self):
        print(self.getWeightMatrix())

    # Returns the Weight Matrix
    # If file is specified, writes it into file as well
    def getWeightMatrix(self, file=None):
        numCx = self.numCx
        numAxons = self.numAxons
        synapses = self.board.nxDriver.synapseCompiler.decodeSynapseMem(
            self.board.n2Chips[0].n2Cores[0])
        weightMatrix = np.zeros((numAxons*numCx), dtype=np.int)
        for i in range(numAxons*numCx):
            weightMatrix[i] = synapses[i].Wgt
        if file:
            wgtFile = open(file, 'w')
            np.savetxt(wgtFile, weightMatrix)
            wgtFile.close()
        return weightMatrix

    def shutdownNetwork(self):
        if self.board:
            self.board.disconnect()
