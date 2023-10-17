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
from nxsdk_modules.slic.src.nxcore.slicnet import SlicNet


class SingleLayerImageClassifier:
    def __init__(self,
                 trainLabelsFile=os.path.dirname(os.path.realpath(
                     __file__))+"/../../data/train_labels.binary",
                 trainImagesFile=os.path.dirname(os.path.realpath(
                     __file__))+"/../../data/train_images.binary",
                 inferLabelsFile=os.path.dirname(os.path.realpath(
                     __file__))+"/../../data/infer_labels.binary",
                 inferImagesFile=os.path.dirname(os.path.realpath(
                     __file__))+"/../../data/infer_images.binary",
                 numSegments=4,
                 bytesPerImage=50,
                 sizeOfImageLabels=4,
                 debug=False):

        # Set this to true to see debug outputs
        self.debug = debug
        self.bytePerImage = bytesPerImage
        self.sizeOfImageLabels = sizeOfImageLabels

        # Setting the file path for training and inference image set
        self.trainLabelsFile = trainLabelsFile
        self.trainImagesFile = trainImagesFile
        self.inferLabelsFile = inferLabelsFile
        self.inferImagesFile = inferImagesFile

        self.trainLabelData, self.trainImageData = self.getLabelAndImageData(
            self.trainLabelsFile, self.trainImagesFile)
        self.inferLabelData, self.inferImageData = self.getLabelAndImageData(
            self.inferLabelsFile, self.inferImagesFile)

        self.model = SlicNet(
            self.trainLabelData,
            self.trainImageData,
            self.inferLabelData,
            self.inferImageData,
            bytesPerImage=50,
        )

    def getLabelAndImageData(self, labelFile, imageFile):
        try:
            labelsFile = open(labelFile, "rb")
            imagesFile = open(imageFile, "rb")
        except IOError:
            print("Cannot Open Label Files or Image Files")
            raise
        labelData = []
        imageData = []

        for label in iter(lambda: labelsFile.read(self.sizeOfImageLabels), b''):
            labelData.append(int.from_bytes(label, byteorder='little'))
            for image in iter(lambda: imagesFile.read(1), b''):
                imageData.append(int.from_bytes(image, byteorder='little'))
        labelsFile.close()
        imagesFile.close()
        return labelData, imageData

    def setupModel(self):
        self.model.setupNetwork()

    def run(self):
        self.model.runTraining()
        self.model.runInference()


if __name__ == "__main__":
    imageClassifier = SingleLayerImageClassifier()
    imageClassifier.setupModel()
    imageClassifier.run()
