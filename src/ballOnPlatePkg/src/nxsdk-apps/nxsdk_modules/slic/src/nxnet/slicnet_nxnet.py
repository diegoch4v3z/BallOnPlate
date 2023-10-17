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
import numpy as np
import nxsdk.api.n2a as nx
from nxsdk.graph.monitor.probes import IntervalProbeCondition
from nxsdk.net.net import CompartmentPrototype
from nxsdk_modules.slic.src.nxnet.linescanprocess import LineScanProcess
from nxsdk_modules.slic.src.nxnet.modelparams import ModelParams


class SlicNet:
    """
    SlicNet configures a single layer of 'n' neurons to classify
    objects into n classes.
    """

    def __init__(self,
                 trainLabelData,
                 trainImageData,
                 inferLabelData,
                 inferImageData,
                 xImgSize=20,
                 yImgSize=20,
                 threshold=128,
                 numSegments=4,
                 modelParams=ModelParams(),
                 debug=False):
        """
        Initializes the SlicNet

        :param trainLabelData: Training label data in the format (numImage, xPixelData, yPixelData)
        :param trainImageData: Training image data in the format (numImage, data)
        :param inferLabelData: Inference label data in the format (numImage, xPixelData, yPixelData)
        :param inferImageData: Inference image data in the format (numImage, data)
        :param numSegments: Number of segments a line is divided into,
                            by the linescanner. Each segment corresponds
                            to an axon.
        :param modelParams: ModelParam class object consisting of model parameters
        :param xImgSize: Size of input image in X direction, in pixels
        :param yImgSize: Size of input image in y direction, in pixels
        :param threshold: Threshold in pixel value for binarization
        :param debug: To toggle debug
        """
        self.net = nx.NxNet()
        self.trainingPhase = 1
        self.modelParams = modelParams
        # Initializing The Model Parameters
        self.currentTime = 1
        self.numInferImages = 0
        self.numCorrect = 0
        self.xImgSize = xImgSize
        self.yImgSize = yImgSize
        self.bytesPerImage = (self.xImgSize * self.yImgSize) // 8
        self.threshold = threshold
        self.trainLabelData = trainLabelData
        self.trainImageData = trainImageData
        self.inferImageDataSet = inferImageData
        self.inferLabelDataSet = inferLabelData
        self._processTrainingLabelsAndImages()
        self.debug = debug
        self.numLines = 18 * \
            (max(self.xImgSize, self.yImgSize) +
             max(self.xImgSize, self.yImgSize) // 2) - 32
        self.numAxons = self.numLines * numSegments

        self.lineScanGen = LineScanProcess(
            self.net, self.numLines, numSegments)

        self.probeActivity = None
        self.probeSpikes = None
        self.probeCxU = None
        self.probeCxV = None

    def _createInitSnip(self):
        """
        Creates a process in init phase, used to receive model parameters.
        """
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/initsnip.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "initsnip"
        guardName = None
        phase = "init"
        self.initProcess = self.board.createProcess(
            "initProcess", cPath, includeDir, funcName, guardName, phase)

    def _createLineSpikingSnip(self):
        """
        Creates a process in spiking phase to execute the linespiking algorithm.
        """
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/linespiking.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "run_spiking"
        guardName = "do_spiking"
        phase = "spiking"
        self.spikingProcess = self.board.createProcess(
            "spikingProcess", cPath, includeDir, funcName, guardName, phase)

    def _createRunMgmtSnip(self):
        """
        Creates a process in the runMgmg phase. This snip is used to :
         a) switch the learning rule for neurons
         b) clear activity probes after each processing each image
            during inference
        """
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "run_mgmt"
        guardName = "do_run_mgmt"
        phase = "mgmt"
        self.runMgmtProcess = self.board.createProcess(
            "runMgmt", cPath, includeDir, funcName, guardName, phase)

    def _createChannels(self):
        """
        Creates the channels :

        initChannel : To send the model parameters
        imageChannel : To send the image data
        labelChannel : To send the label data
        """
        # Create a channel named initChannel for sending the network parameters
        self.initChannel = self.board.createChannel(b'nxinit', "int", 30)
        # Connecting imageChannel from initProcess to SuperHost making it
        # receive channel
        self.initChannel.connect(None, self.initProcess)
        # Create a channel named imageChannel for sending image data
        self.imageChannel = self.board.createChannel(
            b'nximage', "int", self.numTrainImages * self.bytesPerImage)
        # Connecting imageChannel from Superhost to initProcess
        self.imageChannel.connect(None, self.initProcess)
        # Create a channel named labelChannel for sending label data
        self.labelChannel = self.board.createChannel(
            b'nxlabel', "int", self.numTrainImages * self.bytesPerImage)
        # Connecting labelChannel from Superhost to runMgmtProcess
        self.labelChannel.connect(None, self.runMgmtProcess)
        self.board.start()

    def _sendNetworkConfiguration(self):
        """
        Sends the model paramters to Lakemont using initChannel
        """
        self.initChannel.write(1, [self.numCx])
        self.initChannel.write(1, [self.modelParams.numTrainIterations])
        self.initChannel.write(1, [self.modelParams.imagePresentOffset])
        self.initChannel.write(1, [self.numTrainImages])

    def _binarizeImage(self, imageData, threshold):
        """
        Binarizes the image on the basis of the threshold.
        Anything above threshold is set to 1 and below to 0.

        :param imageData: Image dataset to be binarized
        :param threshold: Threshold for binarization
        :return:
        """
        imageData = (imageData > threshold).astype(np.int_)
        return imageData

    def _compress(self, imageData):
        compressedData = []
        for byte in range(0, len(imageData) // 8):
            data = 0
            for bit in range(byte * 8, (byte + 1) * 8):
                data = data | imageData[bit] << (bit - (byte * 8))
            compressedData.append(data)
        return compressedData

    def _processImageAndLabels(self, imageData, labelData):
        """
        Helper function used for pre-processing of dataset.
        In the pre-processing phase, the image is binarized
        based on the threshold configured and then compressed
        into bytes. It converts both image and label dataset into
        python list.

        :param imageData: Image dataset to be processed
        :param labelData: Corresponding label dataset
        :return: tuple of processed imageData, labelData and numImage
        """
        numImage = imageData.shape[0]
        imageData = self._binarizeImage(imageData, self.threshold)
        imageData = np.reshape(
            imageData, (np.product(
                imageData.shape),)).astype(
            np.uint8).tolist()
        imageData = self._compress(imageData)
        labelData = labelData.astype(np.uint8).tolist()
        return imageData, labelData, numImage

    def _processTrainingLabelsAndImages(self):
        """
        Helper function to pre-process training dataset.
        From the training label dataset, it infers number
        of classes thus neurons required for classification.
        """
        print("Processing Training Images")
        self.numCx = len(np.unique(self.trainLabelData))
        self.trainImageData, self.trainLabelData, self.numTrainImages = self._processImageAndLabels(
            self.trainImageData, self.trainLabelData, )
        if self.numTrainImages > self.modelParams.maxNumImages:
            self.numTrainImages = self.modelParams.maxNumImages
        print("Training {} images ".format(self.numTrainImages))

    def _processInferenceLabelsAndImages(
            self, imageData=None, imageLabel=None):
        """
        Helper function to pre-process inference dataset

        :param imageData: Image dataset to process
        :param imageLabel: Label datset corresponding to image
        """
        if not imageData and not imageLabel:
            imageData = self.inferImageDataSet
            imageLabel = self.inferLabelDataSet
        self.inferImageDataSet, self.inferLabelDataSet, _ = self._processImageAndLabels(
            imageData, imageLabel)

    def _createSnipsAndChannels(self):
        """
        Helper function that creates snips and channels
        """
        print("Creating Snips and Channels")
        self._createInitSnip()
        self._createLineSpikingSnip()
        self._createRunMgmtSnip()
        self._createChannels()
        self._sendNetworkConfiguration()

    def _getInferenceImageAndLabel(self):
        """
        Helper function to get an image and label from inference dataset
        :return: 0 if it couldn't get an image othewise 1
        """
        try:
            self.inferImageData = self.inferImageDataSet[self.numInferImages * self.bytesPerImage:(
                self.numInferImages + 1) * self.bytesPerImage]
            self.inferLabelData = self.inferLabelDataSet[self.numInferImages]
            if len(self.inferImageData) != self.bytesPerImage:
                return 0
        except IndexError:
            return 0
        return 1

    def _checkInferenceAccuracy(self, correctLabel=None):
        """
        Function to check if the image is inferred correctly.
        :return: Predicted label
        """
        if self.debug:
            print("Checking for Inference Accuracy at time :{}".format(
                self.currentTime))
        maxSpikes = 0
        for i in range(self.numCx):
            if max(
                    self.probeActivity[i].data[-self.modelParams.timestepPerImage:]) >= maxSpikes:
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

    def _runInferenceOnImage(self, imageData, label):
        """
        Helper function to run inference provided image and actual label

        :param imageData: Image data on which to run inference
        :param label: Actual label
        :return: Predicted label
        """
        if len(imageData) != self.bytesPerImage:
            raise ValueError("Image of Wrong Size Provided")
        self.imageChannel.write(len(imageData), imageData)
        self.board.run(self.modelParams.timestepPerImage)
        self.currentTime += self.modelParams.timestepPerImage
        label = self._checkInferenceAccuracy(label)
        self.numInferImages += 1
        return label

    def configureCx(self,
                    compartmentCurrentDecay=410,
                    compartmentVoltageDecay=410,
                    refractoryDelay=2,
                    enableSpikeBackprop=1,
                    enableSpikeBackpropFromSelf=1,
                    vThMant=9000,
                    vMinExp=23,
                    vMaxExp=23,
                    enableHomeostasis=1,
                    minActivity=0,
                    maxActivity=127,
                    homeostasisGain=0,
                    activityImpulse=1,
                    activityTimeConstant=1000000,
                    cxPrototype=None
                    ):
        """

        :param compartmentCurrentDecay: Decay constant by which the compartmentCurrent decays per
                                        time step.
        :param compartmentVoltageDecay: Decay constant by which the compartmentVoltage decays per
                                        time step.
        :param refractoryDelay: After a compartment spikes it enters a refractory state of duration
                                refractoryDelay during which compartmentVoltage does not accumulate
                                new inputs.
        :param enableSpikeBackprop: Enables spike backpropagation mechanism.
        :param enableSpikeBackpropFromSelf: Enables spike backpropagation when compartment spikes
                                            itself.
        :param vThMant: Threshold voltage of the compartment.
        :param vMinExp: vMinExp determines the minimum voltage vMin at which compartmentVoltage saturates.
        :param vMaxExp: vMaxExp determines the minimum voltage vMin at which compartmentVoltage saturates.
        :param enableHomeostasis: enableHomeostasis determines whether vThMant gets updated via activity
                                  driven threshold homeostasis.
        :param minActivity: minActivity is the lower threshold below which vThMant gets decreased to
                            increase the compartment activity.
        :param maxActivity: maxActivity is the upper threshold above which vThMant gets increased to
                            reduce the compartment activity
        :param homeostasisGain: homeostasisGain controls how strongly vThMant gets modified by activity
                                leaving the range defined by minActivity, maxActivity.
        :param activityImpulse: activitiyImpulse specifies the amount by which activity increases for
                                each compartment spike.
        :param activityTimeConstant: Decay time constant by which activity decays approximately exponentially
                                     over time.
        :param cxPrototype: Compartment Prototype, if provided all compartments created will have this prototype.
                            All the other parameters will be ignored.
        """
        if not cxPrototype:
            cxPrototype = nx.CompartmentPrototype(
                compartmentCurrentDecay=compartmentCurrentDecay,
                compartmentVoltageDecay=compartmentVoltageDecay,
                refractoryDelay=refractoryDelay,
                enableSpikeBackprop=enableSpikeBackprop,
                enableSpikeBackpropFromSelf=enableSpikeBackpropFromSelf,
                vThMant=vThMant,
                vMinExp=vMinExp,
                vMaxExp=vMaxExp,
                enableHomeostasis=enableHomeostasis,
                minActivity=minActivity,
                maxActivity=maxActivity,
                homeostasisGain=homeostasisGain,
                activityImpulse=activityImpulse,
                activityTimeConstant=activityTimeConstant
            )
        else:
            if not isinstance(cxPrototype, CompartmentPrototype):
                raise ValueError(
                    "cxPrototype should be of type CompartmentPrototype")
        self.cxGroup = self.net.createCompartmentGroup(
            size=self.numCx, prototype=cxPrototype)
        # Cx To create the Other Learning Rule
        self.dummyCx = self.net.createCompartment(cxPrototype)

    def configureConnectivity(
            self,
            trainEpoch=2,
            depressingLr=None,
            potentiatingLr=None):
        """
        Configures the connectivity and learning rule

        :param trainEpoch: Length of training epoch
        :param depressingLr: Learning rules to depress the neurons
        :param potentiatingLr: Learning rules to potentiate the neurons
        """
        if not depressingLr:
            depressingLr = self.net.createLearningRule(dw='-2^-5*y0*x1',
                                                       x1Impulse=127,
                                                       x1TimeConstant=10,
                                                       tEpoch=trainEpoch
                                                       )
        if not potentiatingLr:
            potentiatingLr = self.net.createLearningRule(
                dw='-2^-6*y0*x1+2^-5*u2*x1',
                x1Impulse=127,
                x1TimeConstant=10,
                tEpoch=trainEpoch)

        connProtoLr1 = nx.ConnectionPrototype(weight=0,
                                              delay=0,
                                              enableLearning=1,
                                              learningRule=depressingLr)

        connProtoLr2 = nx.ConnectionPrototype(weight=0,
                                              delay=0,
                                              enableLearning=1,
                                              learningRule=potentiatingLr)

        self.ConnGroup = self.lineScanGen.connect(
            self.cxGroup, prototype=connProtoLr1)
        dummyConnGroup = self.dummyCx.connect(self.dummyCx, connProtoLr2)

    def setupCxStateProbes(self):
        """
        Sets up the CxState probes : probes for Compartment Current(u) and Voltage(v)
        """
        probeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                           nx.ProbeParameter.COMPARTMENT_VOLTAGE]
        CxProbes = self.cxGroup.probe(probeParameters)
        self.probeCxU = []
        self.probeCxV = []
        for probe in CxProbes[0]:
            self.probeCxU.append(probe)
        for probe in CxProbes[1]:
            self.probeCxV.append(probe)

    def setupActivityProbes(self):
        """
        Sets up the activity probes
        """
        print("Setting up Activity Probes")
        # Start probing only during inference
        probeStart = self.modelParams.timestepPerImage * self.modelParams.numTrainIterations \
            * self.numTrainImages + self.modelParams.imagePresentOffset + 1
        probeParameters = [nx.ProbeParameter.SOMA_STATE_ACTIVITY]
        CxProbes = self.cxGroup.probe(
            probeParameters,
            probeConditions=IntervalProbeCondition(
                dt=1,
                tStart=probeStart))
        self.probeActivity = []
        for probe in CxProbes[0]:
            self.probeActivity.append(probe)

    def setupSpikeProbes(self):
        """
        Sets up the Spike probes
        """
        probeParameters = [nx.ProbeParameter.SPIKE]
        CxProbes = self.cxGroup.probe(probeParameters)
        self.probeSpikes = []
        for probe in CxProbes[0]:
            self.probeSpikes.append(probe)

    def compileNetwork(self):
        """
        Compiles the network

        :param debug: Toggle debug
        """
        print("Compiling the Network")
        if not self.probeActivity:
            self.setupActivityProbes()
        compiler = nx.N2Compiler()
        self.board = compiler.compile(self.net)
        self._createSnipsAndChannels()

    def sendTrainingImageData(self, start, end):
        """
        Sends image data to the Lakemont using imageChannel

        :param start: Index of the start of data
        :param end: Index of end of data
        """
        self.imageChannel.write(
            end - start + 1, self.trainImageData[start:(end + 1)])

    def sendTrainingLabelData(self, start, end):
        """
        Sends label data to the Lakemont using labelChannel

        :param start: Index of the start of data
        :param end: Index of end of data
        """
        self.labelChannel.write(
            end - start + 1, self.trainLabelData[start:(end + 1)])

    def runTraining(self):
        """
        Runs training on provided trainImageData and trainLabelData
        """
        print("Num iterations is : ", self.modelParams.numTrainIterations)
        for i in range(self.modelParams.numTrainIterations):
            self.sendTrainingImageData(
                start=0, end=len(
                    self.trainImageData) - 1)
            self.sendTrainingLabelData(
                start=0, end=len(
                    self.trainLabelData) - 1)
            self.board.run(self.modelParams.imagePresentOffset +
                           (self.numTrainImages *
                            self.modelParams.timestepPerImage -
                            1))
            self.currentTime += self.modelParams.imagePresentOffset +\
                (self.numTrainImages * self.modelParams.timestepPerImage)
            print("Completed Iteration : ", i)
        print("Training Complete Now. Time for Inference")
        if self.debug:
            self.getWeightMatrix(file="/tmp/wgtMatrix.out")

    def runTrainingInBatch(self, batchSize):
        """
        Runs training in `batches`

        :param batchSize: Size the batch
        """
        print("Num iterations is : ", self.modelParams.numTrainIterations)
        for i in range(self.modelParams.numTrainIterations):
            for batchNum in range(self.numTrainImages // batchSize):
                self.sendTrainingImageData(
                    start=batchNum *
                    batchSize *
                    self.xImgSize *
                    self.yImgSize,
                    end=(
                        batchNum +
                        1) *
                    batchSize *
                    self.xImgSize *
                    self.yImgSize -
                    1)
                self.sendTrainingLabelData(
                    start=batchNum *
                    batchSize,
                    end=(
                        batchNum +
                        1) *
                    batchSize -
                    1)
                self.board.run(self.modelParams.imagePresentOffset +
                               (batchSize *
                                self.modelParams.numTrainIterations *
                                self.modelParams.timestepPerImage) -
                               1)
                self.currentTime += self.modelParams.imagePresentOffset + \
                    (batchSize * self.modelParams.numTrainIterations *
                     self.modelParams.timestepPerImage)
                print("Training Complete For BatchNum : {}".format(batchNum))
            print("Training Complete Now. Time for Inference")
        if self.debug:
            self.getWeightMatrix(file="/tmp/wgtMatrix.out")

    def runInference(self, inferenceImageData=None, inferenceLabelData=None):
        """
        Runs inference on the inferenceImageData and inferenceLabelData.
        If inferenceImageData and inferenceLabelData are not provided it
        runs inference on the inferenceData and inferenceLabelData provided
        during the initialization of the network.

        :param inferenceImageData:
        :param inferenceLabelData:
        :returns: Tuple of predictedlabels(list) and accuracy
        """
        self.trainingPhase = 0
        self._processInferenceLabelsAndImages(
            inferenceImageData, inferenceLabelData)
        labels = []
        while(self._getInferenceImageAndLabel()):
            label = self._runInferenceOnImage(
                self.inferImageData, self.inferLabelData)
            labels.append(label)
            print("Completed Inferring {} Image".format(self.numInferImages))
        accuracy = self.numCorrect / self.numInferImages
        print("Inference Complete : Accuracy is {}".format(accuracy))
        return labels, accuracy

    def printSpikeProbes(self):
        """
        Prints the values of `Spike` Probes for the **last** image.
        `Spike` Probes should be set before using this.
        """
        if not self.probeSpikes:
            raise EnvironmentError("Spike Probes have not been set")
        for i in range(self.numCx):
            print("Spike Probe : {} is {}".format(
                i, self.probeSpikes[i].data[-self.modelParams.timestepPerImage:]))

    def printActivityProbes(self):
        """
        Prints the values of `Activity` Probes for the **last** image.
        `Activity` Probes should be set before using this.
        """
        if not self.probeActivity:
            raise EnvironmentError("Activity Probes have not been set")
        for i in range(self.numCx):
            print("Spike Probe : {} is {}".format(
                i, self.probeActivity[i].data[-self.modelParams.timestepPerImage:]))

    def printCxUProbes(self):
        """
        Prints the values of `CxUProbes` for the **last** image.
        `CxUProbes` should be set before using this.
        """
        if not self.probeCxU:
            raise EnvironmentError("Cx Probes have not been set")
        for i in range(self.numCx):
            print("Cx U : {} is {}".format(
                i, self.probeCxU[i].data[-self.modelParams.timestepPerImage:]))

    def printCxVProbes(self):
        """
        Prints the values of `CxVProbes` for **last** image.
        `CxVProbes` should be set before using this.
        """
        if not self.probeCxV:
            raise EnvironmentError("Cx Probes have not been set")
        for i in range(self.numCx):
            print("Cx V : {} is {}".format(
                i, self.probeCxV[i].data[-self.modelParams.timestepPerImage:]))

    def getWeightMatrix(self, file=None):
        """
        Returns weight matrix as numpy array and writes weight matrix to the file
        if file is provided

        :param file: File name to write weight matrix
        :return: Weight matrix as numpy array
        """
        numCx = self.numCx
        numAxons = self.numAxons
        synapses = self.board.nxDriver.synapseCompiler.decodeSynapseMem(
            self.board.n2Chips[0].n2Cores[0])
        weightMatrix = np.zeros((numAxons * numCx), dtype=np.int)
        for i in range(numAxons * numCx):
            weightMatrix[i] = synapses[i].Wgt
        if file:
            wgtFile = open(file, 'w')
            np.savetxt(wgtFile, weightMatrix)
            wgtFile.close()
        return weightMatrix

    def disconnect(self):
        """
        Shutsdown the newtork on the board
        """
        self.board.disconnect()
