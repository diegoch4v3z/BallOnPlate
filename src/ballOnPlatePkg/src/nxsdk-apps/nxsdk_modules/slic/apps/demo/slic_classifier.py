"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express 
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy, 
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are 
expressly stated in the License.
"""

from nxsdk_modules.slic.apps.demo.slicnet import SlicNet

class SlicClassifier:
    """
    Classifier that uses SlicNet for classification
    """
    def __init__(self):
        self.classifier = None

    def train_on_dataset(self, trainDataSet):
        """
        Train the SLIC network on given dataSet
        """
        trainData = []
        trainLabel = []
        for label, imageData in trainDataSet:
            trainData += imageData
            trainLabel.append(label)

        self.shutdown()

        self.classifier = SlicNet(
            trainImageData=trainData,
            trainLabelData=trainLabel,
            bytesPerImage=50,
            numSegments=4)
        self.classifier.runTraining()

    def infer_on_dataset(self, inferenceDataSet):
        """
        Perform inference on the given dataset
        :return: List of inference result
        """
        inference = []
        for label, imageData in inferenceDataSet:
            inference.append(
                (label, self.classifier.runInferenceOnImageDataStream(imageData, label)))
        print("Inference accuracy is : ", self.classifier.getInferenceAccuracy())
        return inference

    def shutdown(self):
        """
        Shutdown the network
        """
        if self.classifier:
            self.classifier.shutdownNetwork()