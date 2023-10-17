###############################################################
# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2018-2021 Intel Corporation.

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

import os
import math
import time
import numpy as np
import imageio as iio
from skimage import filters as skifilters
from skimage import transform as skitransform
#from sklearn.decomposition.dict_learning import DictionaryLearning
import matplotlib.pyplot as plt


# ------------------------------------------------------------------------------
# IO
def _buildPath(dir, file):
    assert os.path.isdir(dir), '<dir> must be a valid directory.'
    path = os.path.join(dir, file)
    assert os.path.isfile(path), '<file> is must be a valid file path.'
    return path


def loadImage(dir, file):
    """Loads an image (jpg, png, ...) from file and checks that file exists.

    :param str dir: Directory of image.
    :param str file: Filename of image.
    :return: image
    :rtype: numpy.ndarray
    """
    path = _buildPath(dir, file)
    return plt.imread(path)


def loadCsv(dir, file):
    """Loads an array in csv format from file and checks that file exists.

    :param str dir: Directory of array.
    :param str file: Filename of array..
    :return: array
    :rtype: numpy.ndarray
    """
    path = _buildPath(dir, file)
    return np.loadtxt(path, delimiter=',')


# ------------------------------------------------------------------------------
# Image processing
# ToDo: I have decomposed preProcessInputImage into centerResizeImg(..) and
# contrastNormalizeImg(..). Replace usage of this function by new ones.
def preProcessInputImage(inimgpath, fin_size=52):
    """Read the input image, resize its smaller side to 'fin_size', crop it to
     fin_size x fin_size square, normalize it with subtractive normalization using
     a Gaussian kernel
     :param inimgpath: (string) path to the input image (JPG/PNG/..)
     :param fin_size: (int) final size of the image """

    # Read the image file using imageio module
    img = iio.imread(inimgpath, as_gray=True)

    # Resize, rescale, and crop to square
    imgshape = img.shape
    smallerside = np.min(imgshape)
    # Smaller side should become fin_size:
    newshape = tuple(np.int_([x * (fin_size / smallerside) for x in imgshape]))
    img = skitransform.resize(
        img, newshape, preserve_range=True, mode='reflect', anti_aliasing=True)
    # Crop to resize
    img = img[0:fin_size, 0:fin_size]

    # Gaussian subtractive normalization
    # Gaussian filter
    # truncation radius = 2.5 sigmas
    gmg = skifilters.gaussian(img, sigma=5, truncate=2.5, preserve_range=True)
    # Subtract the Gaussian filtered image
    rmg = img - gmg
    # Sigma-map for scaling
    sigmamap = np.sqrt(skifilters.gaussian(
        rmg * rmg, sigma=5, truncate=2.5, preserve_range=True))
    c = np.mean(sigmamap)
    sigmamap[sigmamap < c] = c
    # Rescale the result
    smg = rmg / sigmamap

    return np.array(smg)


def findImages(dataDir, fileTypes=('.jpg', '.png')):
    files = []
    for file in os.listdir(dataDir):
        filePath = os.path.join(dataDir, file)
        if os.path.isfile(filePath) and filePath.endswith(fileTypes):
            files.append(filePath)

    return files


def centerResizeImg(img, dx, dy):
    """Resizes and crops image to a (dy x dx) window around the center of the
    image.

    :param numpy.ndarray img: 2D numpy array representing the image to be
    resized and centered.
    :param int dx: Final size in x dimension after resizing.
    :param int dy: Final size in y dimension after resizing.
    :return np.ndarray img: Resized and centered image.
    """
    # Compute new size of image
    dxRatio = dx / img.shape[1]
    dyRatio = dy / img.shape[0]
    maxRatio = max((dxRatio, dyRatio))
    newShape = tuple(np.round([x * maxRatio for x in img.shape]))
    # Resize image
    img = skitransform.resize(img,
                              newShape,
                              preserve_range=True,
                              mode='reflect',
                              anti_aliasing=True)

    # Crop image around center position
    xCtr = math.floor(img.shape[1] / 2)
    yCtr = math.floor(img.shape[0] / 2)
    return img[yCtr - math.floor(dy / 2):yCtr + math.ceil(dy / 2),
               xCtr - math.floor(dx / 2):xCtr + math.ceil(dx / 2)]


def contrastNormalizeImg(img, sigma=5, truncate=2.5):
    """Perform local contrast normalization of image.

    :param numpy.ndarray img: Image to be contrast normalized.
    :param int sigma: Width of Gaussian filter.
    :param float truncate: ?
    """
    # Subtractive normalization with Gaussian filter and truncation radius =
    # truncate * sigma
    gmg = skifilters.gaussian(img,
                              sigma=sigma,
                              truncate=truncate,
                              preserve_range=True)
    # Subtract the Gaussian filtered image
    rmg = img - gmg
    # Sigma-map for scaling
    sigmamap = np.sqrt(skifilters.gaussian(rmg * rmg,
                                           sigma=sigma,
                                           truncate=truncate,
                                           preserve_range=True))
    c = np.mean(sigmamap)
    sigmamap[sigmamap < c] = c
    # Rescale the result
    smg = rmg / sigmamap

    return np.array(smg)


def loadImagesFromDir(files, imgSizeX, imgSizeY, numImg):
    """Loads a random set of images from directory. Expects that all files in
    directory are images"""

    import warnings
    warnings.filterwarnings('error')

    ctr = 0
    rawImages = []
    resImages = np.zeros((imgSizeY, imgSizeX, numImg))
    procImages = np.zeros((imgSizeY, imgSizeX, numImg))
    for imgPath in files:
        try:
            rawImg = iio.imread(imgPath, as_gray=True)
        except:
            print("Failed reading img %d: %s" % (ctr, imgPath))
        rawImages.append(rawImg)
        resImg = centerResizeImg(rawImg, imgSizeX, imgSizeY)
        resImages[:, :, ctr] = resImg
        procImg = contrastNormalizeImg(resImg)
        procImages[:, :, ctr] = procImg
        ctr += 1

    return rawImages, resImages, procImages


def extractPatches(images, dx, dy, numPatchesPerImg):
    """Extracts image patches of size (dy x dx) from random positions of
    image set.

    :param numpy.ndarray images: (imgSizeY, imgSizeX, numImg) numpy array of
    gray-scale images.
    :param int dx: x dimension of patch to extract.
    :param dy: y dimension of patch to extract.
    :param numPatchesPerImg: Number of patches to extract per image.
    :return: (dy, dx, numImg*numPatchesPerImg) patches.
    :rtype: numpy.ndarray
    """
    imgSizeX = images.shape[1]
    imgSizeY = images.shape[0]
    numImg = images.shape[2]
    numImgPatches = numImg * numPatchesPerImg
    imgPatches = np.zeros((dy, dx, numImgPatches))
    p = 0
    for imgIdx in range(numImg):
        for patchIdx in range(numPatchesPerImg):
            x0 = np.random.randint(0, imgSizeX-dx+1)
            y0 = np.random.randint(0, imgSizeY-dy+1)
            imgPatches[:, :, p] = images[y0:y0+dy, x0:x0+dx, imgIdx]
            p += 1

    return imgPatches


# ------------------------------------------------------------------------------
# Dictionary learning
def learnDictionary(dataset, dictSize, regularizationFactor, splitSign=True,
                    numParallelJobs=16, tolerance=1e-8, randomSeed=None,
                    algo='lars'):
    """Learns a sparse coding dictionary from dataset using scikit-learn
    module DictionaryLearning..

    :param numpy.ndarray dataset: (numSamples, numFeatures) training dataset.
    :param int dictSize: Number of dictionary elements or features.
    :param float regularizationFactor: Regularization factor of objective
    function.
    :param bool splitSign: Controls whether to split sparse feature vector
    into the concatenation of its negative part and its positive part. This
    can improve performance.
    :param int numParallelJobs: Number of parallel jobs to run.
    :param float tolerance: Tolerance for numerical error.
    :param int randomSeed: Random seed for random number generator.
    :param str algo: Fit algorithm 'cd' (coordinate descent) or 'lars'. LARS
    will be faster if the estimated components are sparse.

    :return: (dictSize, numFeatures) dictionary
    :rtype: numpy.ndarray
    """
    assert isinstance(dataset, np.ndarray) and len(dataset.shape) == 2, \
        'dataset must be a (numSamples, numFeatures) numpy.ndarray.'
    dl = DictionaryLearning(n_components=dictSize,
                            alpha=regularizationFactor,
                            split_sign=splitSign,
                            fit_algorithm=algo,
                            n_jobs=numParallelJobs,
                            max_iter=2500,
                            tol=tolerance,
                            random_state=randomSeed,
                            verbose=False)
    print('Learning dictionary...')
    print('   Number of features: %d' % (dataset.shape[1]))
    print('   Number of samples: %d' % (dataset.shape[0]))
    print('   Dictionary size: %d' % (dictSize))
    print('   Regularization factor: %d' % (regularizationFactor))
    tStart = time.clock()
    dl.fit(dataset)
    dt = time.clock() - tStart
    print('Done. Number of iterations: %d. Runtime: %ds.' % (dl.n_iter_, dt))

    return dl.components_


def learnDictionaryFromPatches(images, patchSize, numPatchesPerImg,
                               dictSize, regFactor,
                               randomSeed=0, numParallelJobs=10,
                               saveDict=True, path=None):
    """Learns a dictionary from images.
    Note: Uses column-major mode or Fortran style convert 2D patches or
    dictionary elements into 1D vectors because LcaNet currently expects that!
    """
    # Extract random set of patches from images
    patches = extractPatches(images, patchSize, patchSize, numPatchesPerImg)
    # Reshape training patches into (numSamples, numFeatures) matrix
    numFeatures = patchSize**2
    numSamples = patches.shape[2]
    dataset = patches.reshape(numFeatures, numSamples, order='F').transpose()
    # Learn dictionary
    dictionary = learnDictionary(dataset, dictSize, regFactor,
                                 algo='cd', randomSeed=randomSeed,
                                 numParallelJobs=numParallelJobs)

    if saveDict:
        assert isinstance(path, str), 'path must specify a valid dictionary ' \
                                      'directory.'
        np.savetxt(path, dictionary, delimiter=',')

    return dictionary
