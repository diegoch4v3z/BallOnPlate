# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2019-2021 Intel Corporation.

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
from nxsdk_modules.slic.src.nxnet.slicnet_nxnet import SlicNet

class SingleLayerImageClassifier:
    def __init__(self,
                 trainLabelsFile=os.path.dirname(os.path.realpath(
                     __file__)) + "/../../data/trainLabel.npy",
                 trainImagesFile=os.path.dirname(os.path.realpath(
                     __file__)) + "/../../data/trainData.npy",
                 inferLabelsFile=os.path.dirname(os.path.realpath(
                     __file__)) + "/../../data/inferLabel.npy",
                 inferImagesFile=os.path.dirname(os.path.realpath(
                     __file__)) + "/../../data/inferData.npy",
                 debug=False):

        # Set this to true to see debug outputs
        self.debug = debug

        self.trainLabelData, self.trainImageData = self.getLabelAndImageData(
            trainLabelsFile, trainImagesFile)
        self.inferLabelData, self.inferImageData = self.getLabelAndImageData(
            inferLabelsFile, inferImagesFile)

        self.model = SlicNet(
            self.trainLabelData,
            self.trainImageData,
            self.inferLabelData,
            self.inferImageData,
            xImgSize=self.trainImageData.shape[1],
            yImgSize=self.trainImageData.shape[2]
        )

    def getLabelAndImageData(self, labelFile, imageFile):
        try:
            labelData = np.load(labelFile)
            imageData = np.load(imageFile)
        except IOError:
            print("Cannot Open Label Files or Image Files")
            raise
        return labelData, imageData

    def setupNetwork(self):
        print("Setting up the Network")
        self.model.configureCx()
        self.model.configureConnectivity()

    def setupProbes(self):
        self.model.setupActivityProbes()

    def compileNetwork(self):
        self.model.compileNetwork()

    def runTraining(self):
        self.model.runTraining()

    def runInference(self):
        self.model.runInference()


if __name__ == "__main__":
    imageClassifier = SingleLayerImageClassifier()
    imageClassifier.setupNetwork()
    imageClassifier.setupProbes()
    imageClassifier.compileNetwork()
    imageClassifier.runTraining()
    imageClassifier.runInference()
