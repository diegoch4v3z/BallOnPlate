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

"""Dimension reduction module"""

import numpy as np
from sklearn.decomposition import PCA, FastICA, TruncatedSVD, NMF
import os

class DimensionReduction:
    """
    Implements PCA/ICA dimensionality reduction for a dataset. 
    Useful to limit the dimensionality for KNN algorithm to fit in Loihi.
    """
    def __init__(self, encodingMatrix=None, encodingFile=None, dataset=None):
        """
        Initialize the dimension reduction module by either passing an encoding matrix,
        a path to a file containing an encoding matrix, or a dataset from which to compute
        the encoding matrix. If the file path does not exist, it will be written with the 
        computed encoding matrix.
        
        :param ndarray encodingMatrix: Optional, a pre-computed encoding matrix
        :param ndarray encodingFile: Optional, a path to a pre-computed encoding matrix
        :param ndarray dataset: Optional, a dataset from which to computed the encoding matrix
        """ 
        if encodingMatrix is not None:
            self.encoding=encodingMatrix
        elif encodingFile is not None:
            if os.path.isfile(encodingFile):
                self.encoding = np.load(encodingFile)
            elif dataset is not None:
                self.encoding = self._computeEncoding(dataset)
                np.save(encodingFile, self.encoding)
            else:
                assert False, "An encoding matrix, file, or dataset must be used for initialization"
        elif dataset is not None:
            self.encoding = self._computeEncoding(dataset)
        else:
            assert False, "An encoding matrix, file, or dataset must be used for initialization"
            
    def reduce(self, data):
        """
        Applies the encoding to data and returns the result.
        
        :param ndarray data: The data to encode, one sample per row.
        
        :returns: The encoded data, one sample per row
        :rtype: ndarray
        """
        encodedData = np.dot(data, self.encoding.T)
        #if len(encodedData.shape)>1:
        #    encodedData /= np.expand_dims(np.linalg.norm(encodedData, axis=1), axis=1) + np.finfo(float).eps
        return encodedData
    
    def _centerData(self, csData):
        """
        Makes the data zero mean.
        """
        csData -= csData.mean(axis=1)[:, np.newaxis]
        return csData
    
    
    def _computePCA(self, csData, num_components):
        """
        Computes PCA on the data.
        """
        pca_encoder = PCA(num_components)

        pca_coefs = pca_encoder.fit_transform(csData)
        pca_comps = pca_encoder.components_
        pca_eigs = pca_encoder.explained_variance_

        return pca_coefs, pca_comps, pca_eigs
    
    
    def _computeICA(self, pca_coefs, num_components, which_pcs=None):
        """
        Computes ICA on the data.
        """
        if which_pcs is None:
            which_pcs = np.arange(num_components)

        fica_encoder = FastICA(num_components)   
        ica_coefs = fica_encoder.fit_transform(pca_coefs[:, which_pcs])
        ica_comps = fica_encoder.components_

        return ica_coefs, ica_comps
    
    
    def _computeEncoding(self, inputData, num_components=500, numSamples=50000):
        """
        Create a new encoding matrix from the input data.
        """
        numInputSamples = inputData.shape[0]
        if numInputSamples>numSamples:
            print("More than {} samples, only using the first {}".format(numSamples, numSamples))
            csData = inputData[:numSamples,:] 
        else:
            print("Fewer than {} samples, using all {}".format(numSamples, numInputSamples))
            csData = inputData
        
        csData = self._centerData(csData)

        pca_coefs, pca_comps, pca_eigs = self._computePCA(csData, num_components)
        ica_coefs, ica_comps = self._computeICA(pca_coefs, num_components)
        
        ica_comps /= ica_comps.std(axis=0)
        return np.dot(ica_comps, pca_comps[:num_components,:])
