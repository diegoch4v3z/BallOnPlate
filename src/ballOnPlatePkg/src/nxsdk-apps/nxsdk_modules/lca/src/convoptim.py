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

import time
import math
import collections
import numpy as np
import scipy.sparse as scsparse
import matplotlib.pyplot as plt
import spams


class OptimMethod:
    functorsDict = {'LARS': spams.lasso,
                    'FISTA': spams.fistaFlat
                    }
    paramsDict = {'LARS': {'pos': True,
                           'numThreads': 1,
                           'X': '',
                           'D': '',
                           'lambda1': '',
                           'return_reg_path': False,
                           'verbose': False},
                  'FISTA': {'pos': True,
                            'numThreads': 1,
                            'loss': 'square',
                            'regul': 'l1',
                            'W0': '',
                            'Y': '',
                            'X': '',
                            'lambda1': '',
                            'return_optim_info': False,
                            'verbose': False}
                  }
    varnamesDict = {'LARS': {'img': 'X',
                             'dict': 'D',
                             'regCoeff': 'lambda1',
                             'max_iter': 'max_length_path',
                             'return_iter_path': 'return_reg_path'},
                    'FISTA': {'img': 'Y',
                              'dict': 'X',
                              'regCoeff': 'lambda1',
                              'max_iter': 'max_it',
                              'return_iter_path': 'return_optim_info'}
                    }

    def __init__(self, name):
        """Wrapper class to hold attributes of a specific 
        optimization method
        :param str name: Name of the optimization method, viz., 'LARS' or 'FISTA' 
        :param NumPy array dict: Full dictionary (not minimal)
        :param NumPy array img: Complete input image
        :param float regCoeff: Regularization coefficient lambda
        """
        self.name = name
        self.functor = OptimMethod.functorsDict[name]
        self.params = OptimMethod.paramsDict[name]
        self.imgdictlambdaNames = OptimMethod.varnamesDict[name]

    def _setImage(self, img):
        """Set input image"""
        self.params[self.imgdictlambdaNames['img']] = img

    def _setDict(self, dict):
        """Set full dictionary"""
        self.params[self.imgdictlambdaNames['dict']] = dict
        if self.name == 'FISTA':
            self.params['W0'] = np.zeros((dict.shape[1], 1))

    def _setRegCoeff(self, regCoeff):
        """Set regularization coefficient"""
        self.params[self.imgdictlambdaNames['regCoeff']] = regCoeff

    def setNumIterations(self, numIt):
        assert numIt > 0, \
            'Number of iterations must be positive.'
        self.params[self.imgdictlambdaNames['max_iter']] = numIt


# ToDo: Don't duplicate this class but share with LcaNet
class Solution():
    """Represents the solution of a sparse coding problem:
    argmin_a |x - D'*a|**2 + lambda*|a|_1
    It is returned by the solve() method and contains the sparsecode,
    the reconstruction error, the objective and sparsity as a function of time.
    """

    def __init__(self, iterations, sparseCode, reconstruction, error,
                 objective, sparsity, inputImg):
        self.iterations = iterations
        self.sparseCode = sparseCode
        self.reconstruction = reconstruction
        if len(self.reconstruction.shape) == 3:
            self.finalReconstruction = np.copy(self.reconstruction[:, :, -1])
        else:
            self.finalReconstruction = np.copy(self.reconstruction)
        self.error = error
        self.objective = objective
        self.sparsity = sparsity
        self.input = inputImg

    @property
    def time(self):
        """Returns an array with the number of iterations."""
        return self.iterations

    def printMetrics(self):
        """Prints the reconstruction error, objective and sparsity at the
        last time step of the solution."""

        assert isinstance(self.error, np.ndarray) and \
            isinstance(self.objective, np.ndarray) and \
            isinstance(self.sparsity, np.ndarray), 'One of error, objective, or ' \
            'sparsity is not a NumPy array'

        finalError = self.error[-1]
        finalObjective = self.objective[-1]
        finalSparsity = self.sparsity[-1]

        print('Error     = {:.2f}'.format(finalError))
        print('Objective = {:.2f}'.format(finalObjective))
        print('Sparsity  = {:.2f}%'.format(finalSparsity))

    @staticmethod
    def makeColorbar(mappable):
        from mpl_toolkits.axes_grid1 import make_axes_locatable
        ax = mappable.axes
        fig = ax.figure
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05)
        return fig.colorbar(mappable, cax=cax)

    def plot(self, figId1=10, figId2=11, iteration=None):
        """Plots temporal evolution of reconstruction error, objective
        and sparsity of the LASSO problem."""

        fig1 = plt.figure(figId1)
        plt.subplot(3, 1, 1)
        plt.plot(self.time, self.error)
        plt.ylabel('Reconstruction error')

        plt.subplot(3, 1, 2)
        plt.plot(self.time, self.objective)
        plt.ylabel('Objective')

        plt.subplot(3, 1, 3)
        plt.plot(self.time, self.sparsity)
        plt.xlabel('Time (in # of time-steps)')
        plt.ylabel('Sparsity (%)')

        # Plot reconstruction
        plt.figure(figId2)
        ax1 = plt.subplot(1, 2, 1)
        ax1.xaxis.set_visible(False)
        ax1.yaxis.set_visible(False)
        img1 = plt.imshow(self.input, cmap='gray', aspect='equal')
        plt.title('Original')
        self.makeColorbar(img1)

        if iteration is None:
            iteration = self.time[-1]

        ax2 = plt.subplot(1, 2, 2)
        ax2.xaxis.set_visible(False)
        ax2.yaxis.set_visible(False)
        img2 = plt.imshow(self.reconstruction[:, :, iteration],
                          cmap='gray', aspect='equal')
        plt.title('Reconstruction')
        self.makeColorbar(img2)

        fig1.tight_layout()
        plt.show()

    def showReconstruction(self):
        plt.figure(1)
        plt.subplot(1, 2, 1)
        plt.imshow(self.input, aspect='auto')
        plt.colorbar()
        plt.subplot(1, 2, 2)
        plt.imshow(self.finalReconstruction, aspect='auto')
        plt.colorbar()
        plt.show()


class ConvOptim:
    def __init__(self, modelParams, optimMethod):
        """A unified class for generating sparse code and reconstruction 
        using that sparse code, with conventional optimization methods 
        like LARS and FISTA algorithms. 

        For LARS and FISTA, this class uses SPAMS modules developed at 
        INRIA (http://spams-devel.gforge.inria.fr)
        """
        self.optimMethod = optimMethod
        self.modelParams = modelParams
        self.rawinput = None

    def _patchToIdx(self):
        """Convert patch ID to indices in image"""
        mp = self.modelParams

        numPatchesX = 1
        numPatchesY = 1
        if mp.strideSizeX != 0:
            numPatchesX = (mp.imgSizeX - mp.patchSizeX) / mp.strideSizeX + 1
        if mp.strideSizeY != 0:
            numPatchesY = (mp.imgSizeY - mp.patchSizeY) / mp.strideSizeY + 1
        numPatches = np.int_(numPatchesX * numPatchesY)

        idxMap = np.reshape(np.array(
            range(mp.imgSizeX * mp.imgSizeY)), (mp.imgSizeY, mp.imgSizeX), order='F')
        numStepsX = numPatchesX
        numStepsY = numPatchesY
        patchToIdxMap = np.zeros(
            (numPatches, np.int_(mp.patchSizeX * mp.patchSizeY)))
        for p in range(numPatches):
            py = np.mod(p, numStepsY)  # First: iterate rows
            px = np.int_(np.floor(p / numStepsX))  # Second: iterate columns
            xIdx = np.array(range(np.int_(px * mp.strideSizeX),
                                  np.int_(px * mp.strideSizeX + mp.patchSizeX)))
            yIdx = np.array(range(np.int_(py * mp.strideSizeY),
                                  np.int_(py * mp.strideSizeY + mp.patchSizeY)))
            nIdx = np.transpose(
                idxMap[np.ix_(xIdx, yIdx)])  # Individual patch needs to be transposed to match solution vector
            patchToIdxMap[p, :] = np.transpose(np.reshape(
                nIdx, (nIdx.shape[0] * nIdx.shape[1], 1), order='F'))

        return patchToIdxMap

    def _separateDictionary(self, dict):
        """Separates (n x m) dictionary into positive only (2n x 2m)
        dictionary:
            D -> D+  D-
                 D-  D+
        Where D+ and D- are given by:
            D+ = D(D>=0)
            D- = -D(D<0)
        """
        n, m = dict.shape

        dp = np.zeros((n, m), dtype=float)
        dm = np.zeros((n, m), dtype=float)
        idxp = dict >= 0
        idxm = dict < 0
        dp[idxp] = dict[idxp]
        dm[idxm] = -dict[idxm]

        d = np.zeros((2 * n, 2 * m), dtype=float)
        d[0:n, 0:m] = dp
        d[n:2 * n, m:2 * m] = dp
        d[0:n, m:2 * m] = dm
        d[n:2 * n, 0:m] = dm

        return d

    def _expandDictionary(self, dict):
        """Expand minimal +/- dictionary to full dictionary
        """
        mp = self.modelParams

        D = np.transpose(self._separateDictionary(dict))

        numPatchesX = 1
        numPatchesY = 1
        if mp.strideSizeX != 0:
            numPatchesX = (mp.imgSizeX - mp.patchSizeX) / mp.strideSizeX + 1
        if mp.strideSizeY != 0:
            numPatchesY = (mp.imgSizeY - mp.patchSizeY) / mp.strideSizeY + 1
        numPatches = np.int_(numPatchesX * numPatchesY)
        expandedDictShape = (2 * mp.imgSizeX * mp.imgSizeY,
                             2 * mp.dictSize * numPatches)
        expandedDict = np.zeros(np.int_(expandedDictShape))

        patchToIdxMap = self._patchToIdx()

        for p in range(numPatches):
            # Pixel indices correspond to neuron indices
            nIdx1 = patchToIdxMap[p, :]
            nIdx2 = np.int_(nIdx1 + expandedDictShape[0] / 2)
            # Copy neuron indices to second half
            nIdx = np.int_(np.concatenate((nIdx1, nIdx2)))
            pIdx = np.int_(
                np.array(range(2 * p * mp.dictSize, 2 * (p + 1) * mp.dictSize)))
            # Copy single patch dictionary into full dictionary
            expandedDict[np.ix_(nIdx, pIdx)] = D

        return np.asfortranarray(expandedDict)

    def setDictionary(self, dict):
        om = self.optimMethod
        omdict = self._expandDictionary(dict)
        om._setDict(omdict)

    def _separatePosNegImg(self, image):
        """Separate out positive and negative parts of the 
        input image and concatenate them together"""

        n, m = image.shape
        image = np.reshape(image.T, (n * m, 1))

        posImg = np.copy(image)
        negImg = -np.copy(image)

        posImg[image < 0] = 0
        negImg[image >= 0] = 0

        return np.asfortranarray(np.concatenate((posImg, negImg), axis=0))

    def setInput(self, img):
        om = self.optimMethod
        self.rawinput = img
        omimg = self._separatePosNegImg(img)
        om._setImage(omimg)

    def setRegCoeff(self, regCoeff):
        om = self.optimMethod
        om._setRegCoeff(regCoeff)

    def _computeMetrics(self, reconstruction, sparsecode, iterations=1):

        om = self.optimMethod

        img = om.params[om.imgdictlambdaNames['img']]
        regCoeff = om.params[om.imgdictlambdaNames['regCoeff']]

        error = np.sqrt(np.sum((img - reconstruction) *
                               (img - reconstruction), axis=0))

        objective = 0.5 * error * error + regCoeff * \
            np.sum(np.abs(sparsecode), axis=0)

        sparsity = np.sum(np.abs(sparsecode) > 0, axis=0) / \
            sparsecode.shape[0] * 100

        m, _ = reconstruction.shape
        reconstruction = reconstruction[0:int(m / 2), :] \
            - reconstruction[int(m / 2):m, :]
        numSteps = reconstruction.shape[1]
        reconstruction = reconstruction.reshape(
            (int(np.sqrt(m / 2)), int(np.sqrt(m / 2)), numSteps),
            order='F')

        return Solution(iterations, sparsecode, reconstruction,
                        error, objective, sparsity, self.rawinput)

    def solve(self):
        """Run the appropriate SPAMS routine and return a collated Solution class
        :param string method: the SPAMS routine to run
        :param NumPy array img: input image to represent with a sparse code
        :param NumPy array dic: dictionary
        :param float regCoeff: regularization coefficient `lambda'
        :returns 
        """

        om = self.optimMethod
        fn = om.functor
        pm = om.params

        print('\n\tRunning ' + om.name + '... ', end='', flush=True)
        starttime = time.time()
        sparsecode = fn(**pm)
        runtime = time.time()
        print('Done ({:.2f}s)'.format(runtime - starttime))

        print('\tReconstructing... ', end='', flush=True)
        dict = om.params[om.imgdictlambdaNames['dict']]
        starttime = time.time()
        sparsedict = scsparse.csr_matrix(dict)
        reconstruction = sparsedict.dot(sparsecode)
        runtime = time.time()
        print('Done ({:.2f}s)'.format(runtime - starttime))
        if scsparse.issparse(reconstruction):
            # Explicit np.array type conversion is required below,
            # because '*' operator for NumPy arrays gives element-wise
            # product, whereas the same operator means a dot-product
            # for SciPy arrays ('*' operator is used to compute the
            # reconstruction error using the reconstruction computed
            # below.
            reconstruction = np.array(reconstruction.todense())

        if scsparse.issparse(sparsecode):
            sparsecode = np.array(sparsecode.todense())

        solution = self._computeMetrics(reconstruction, sparsecode)

        return solution

    def _run(self, numIt):
        """Runs solver for numIt iterations and returns final sparse code.

        :param int numIt: Number of iterations to run.
        :return numpy.ndarray sc: Final sparse code.
        :return int rt: Runtime
        """

        fn = self.optimMethod.functor
        pm = self.optimMethod.params

        self.optimMethod.setNumIterations(numIt)
        startTime = time.time()
        sc = fn(**pm)
        endTime = time.time()
        rt = endTime - startTime

        return sc, rt

    def _runIterDumb(self, numIt, itIncBase):
        """Runs solver for numIt iterations and returns trace of sparse code
        during convergence.
        Due to an apparent bug in the spams.lasso function, it does not
        return the full trace of the sparse code natively when
        return_reg_path = True. As a workaround this function executes the
        the solver with return_reg_path = False multiple times. The number of
        iterations for each intermediate run increase exponentially from 2,
        4, 8, ... numIt.

        :param int numIt: Number of iterations to run.
        :return numpy.ndarray iterations: Number of iterations of each
        individual run.
        :return numpy.ndarray scTrace: Trace of sparse code during convergence.
        :return int rt: Runtime of last iteration for numIt iterations.
        """

        om = self.optimMethod
        fn = om.functor
        pm = om.params

        # Generate exponentially increasing number of intermediate iterations
        iterations = [2]
        i = 1
        while iterations[i-1] < numIt:
            iterations.append(
                min(math.ceil(iterations[i-1]*itIncBase), numIt))
            i += 1
        iterations = np.asarray(iterations, dtype=int)

        # Run solver multiple times for increasing number of iterations
        sc = []
        for i in iterations:
            om.setNumIterations(int(i))
            isLastIteration = i == iterations[-1]
            if isLastIteration:
                startTime = time.time()
            spcd = fn(**pm)
            if isLastIteration:
                runtime = time.time() - startTime
            if scsparse.issparse(spcd):
                spcd = np.array(spcd.todense())
            sc.append(spcd)

        return iterations, np.asarray(np.hstack(sc)), runtime

    def _runIter(self, numIt):
        """Runs solver for numIt iterations and returns trace of sparse code
        during convergence.
        Due to an apparent bug in the spams.lasso function, it does not
        return the full trace of the sparse code natively when
        return_reg_path = True. Instead, the sparse code will just drop to 0
        after some number of iterations that depends on the particular input.

        :param int numIt: Number of iterations to run.
        :return numpy.ndarray iterations: Number of iterations of each
        individual run.
        :return numpy.ndarray scTrace: Trace of sparse code during convergence.
        :return int rt: Runtime of last iteration for numIt iterations.
        """

        om = self.optimMethod
        fn = om.functor
        pm = om.params
        pm[om.imgdictlambdaNames['return_iter_path']] = True

        self.optimMethod.setNumIterations(numIt)
        startTime = time.time()
        scFinal, scTrace = fn(**pm)
        iterations = np.arange(1, numIt + 1)
        endTime = time.time()
        scTrace[:, -1] = scFinal.todense().reshape(scFinal.shape[0])
        rt = endTime - startTime

        return iterations, scTrace, rt

    def solvePerf(self, numIt, numReps, itIncBase=2, reconstruct=False):
        """Run the appropriate SPAMS routine in a loop for numReps number of 
        times. Reconstruction is performed only if reconstruct=True (default: 
        False)
        """

        # Dumb iteration runs solver multiple times with increasing number of
        #  iterations until final numIt as a workaround to zero-sparsecode bug
        useDumbIteration = True

        om = self.optimMethod
        runtimes = np.zeros((numReps))
        solution = collections.namedtuple(
            'Solution', 'meanRuntime, stdRuntime')

        for j in range(numReps):
            print('\n\tRunning ' + om.name + ' rep #{}... '.format(j),
                  end='', flush=True)

            if j == 0:
                #  Record trace of sparse code only during first repetition
                if useDumbIteration:
                    iterations, scTrace, rt = self._runIterDumb(
                        numIt, itIncBase)
                else:
                    iterations, scTrace, rt = self._runIter(numIt)
            else:
                # Measure only runtime during remaining repetitions
                _, rt = self._run(numIt)

            runtimes[j] = rt
            print('Done ({:.2f}s)'.format(rt))

            if reconstruct and j == 0:
                print('\tReconstructing rep #{}... '.format(j),
                      end='', flush=True)
                dict = om.params[om.imgdictlambdaNames['dict']]
                startTime = time.time()
                sparseDict = scsparse.csr_matrix(dict)
                reconstruction = sparseDict.dot(scTrace)
                print('Done ({:.2f}s)'.format(time.time() - startTime))
                if scsparse.issparse(reconstruction):
                    # Explicit np.array type conversion is required below,
                    # because '*' operator for NumPy arrays gives element-wise
                    # product, whereas the same operator means a dot-product
                    # for SciPy arrays ('*' operator is used to compute the
                    # reconstruction error using the reconstruction computed
                    # below.
                    reconstruction = np.array(reconstruction.todense())
                solution = self._computeMetrics(reconstruction, scTrace,
                                                iterations)

        solution.runtimes = runtimes
        solution.avgTimePerRep = np.mean(runtimes)
        solution.stddevTimePerRep = np.std(runtimes)

        return solution

    def solveErg(self, numIt, numReps):
        """Run the appropriate SPAMS routine in a loop for numReps number of
        times for power measurement. This is a lean-routine, which does not
        return anything. It is just supposed to solve the LASSO problem.
        """

        for j in range(numReps):
            # Measure only runtime during remaining repetitions
            sc, rt = self._run(numIt)

        return 0
