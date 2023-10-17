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

import math
from collections import OrderedDict, Iterable

import matplotlib.pyplot as plt
import numpy as np
import networkx as ntx

from nxsdk.logutils.nxlogging import *
from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from nxsdk.graph.monitor.probes import IntervalProbeCondition
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk_modules.lca.src.resource_estimator import ResourceEstimator
from nxsdk_modules.lca.src.groups import NeuroCore, NeuronGroup, \
    ConnectivityGroup
from nxsdk_modules.lca.src.patch import Patch


class ModelParams:
    """Contains all public model parameters for LcaNet and computes derived
    model parameters."""

    def __init__(self):
        # Input related
        self._imgSizeX = None
        self._patchSizeX = None
        self._strideSizeX = None
        self._imgSizeY = None
        self._patchSizeY = None
        self._strideSizeY = None
        self._numStridesX = None
        self._numStridesY = None
        self._regularizationFactor = 0.6
        self._biasScaling = 0.64*(2**4)*2**6  # =tau*2**wgtBits*2**6
        self._biasPrecision = 13   # Includes sign. Only 12 value bits
        # Dictionary related
        self._dictSize = None
        # Neuron/synapse related
        self._decayU = 128  # 1/(tau/dt)*2^^12
        self._decayV = 0
        self._vth = 512  # =2**4 * 0.64/0.02=2**wgtBits/tau/dt
        self._wgtExp = 0
        self._wgtBits = 4
        self._useMixedWeights = 0
        self._enableSomaTrace = 1
        self._tEpochDend = 1
        self._somaSpikeLevelInt = 1
        self._somaSpikeLevelFrac = 0
        self._somaTau = 10000000
        self._useOverlapBasedWgtScaling = 0
        self._maxConnectionDistance = 1e10
        # Reconstruction parameters
        self._timePerStep = 0.020  # in second
        self._skipWindow = 100  # first few time steps to skip for reconstruction
        # Run-time parameters
        self._numEpochs = 4
        self._numStepsPerEpoch = 100
        self._probeInterval = 10
        self._probeOffset = 0
        self._enableSpikes = True
        self._disableOffboardSpikes = False
        self.probeActivity = True
        self.probeCxIdList = None

    # Getter/Setter interface
    @property
    def imgSizeX(self):
        """Size of input image in X direction, in pixels"""
        self._assertIsDefined('_imgSizeX')
        return self._imgSizeX

    @property
    def patchSizeX(self):
        """Size of a patch (or equivalently, a single
        dictionary element) in X direction, in pixels"""
        self._assertIsDefined('_patchSizeX')
        return self._patchSizeX

    @property
    def strideSizeX(self):
        """Stride length in X direction, in pixels"""
        self._assertIsDefined('_strideSizeX')
        return self._strideSizeX

    @property
    def imgSizeY(self):
        """Size of input image in Y direction, in pixels"""
        self._assertIsDefined('_imgSizeY')
        return self._imgSizeY

    @property
    def patchSizeY(self):
        """Size of a patch (or equivalently, a single
        dictionary element) in Y direction, in pixels"""
        self._assertIsDefined('_patchSizeY')
        return self._patchSizeY

    @property
    def strideSizeY(self):
        """Stride length in Y direction, in pixels"""
        self._assertIsDefined('_strideSizeY')
        return self._strideSizeY

    def setImgParamsXY(self, imgSizeX, patchSizeX, strideSizeX,
                       imgSizeY, patchSizeY, strideSizeY):
        """Sets imgSizeX/Y, patchSizeX/Y and strideSizeX/Y for input image.

        :param int imgSizeX/Y: Positive integer specifying size of image in \
        X/Y dimension.
        :param int patchSizeX/Y: Positive integer specifying size of an \
        individual patch in X/Y dimension.
        :param int strideSizeX/Y: Positive integer specifying size a stride \
        from one patch to next in input image space in X/Y dimension.
        """

        self._validatePosInteger(imgSizeX, 'imgSizeX')
        self._validatePosInteger(patchSizeX, 'patchSizeX')
        self._validatePosInteger(strideSizeX+1, 'strideSizeX')
        self._validatePosInteger(imgSizeY, 'imgSizeY')
        self._validatePosInteger(patchSizeY, 'patchSizeY')
        self._validatePosInteger(strideSizeY+1, 'strideSizeY')

        self._numStridesX = ModelParams.computeNumStrides(
            imgSizeX, patchSizeX, strideSizeX)
        self._numStridesY = ModelParams.computeNumStrides(
            imgSizeY, patchSizeY, strideSizeY)
        self._imgSizeX = imgSizeX
        self._patchSizeX = patchSizeX
        self._strideSizeX = strideSizeX
        self._imgSizeY = imgSizeY
        self._patchSizeY = patchSizeY
        self._strideSizeY = strideSizeY

    def setImgParams(self, imgSize, patchSize, strideSize):
        """Same as setImgParamsXY but uses same parameters for X/Y dimension.

        :param int imgSize: Positive integer specifying size of image in \
        X and Y dimension.
        :param int patchSize: Positive integer specifying size of an \
        individual patch in X and Y dimension.
        :param int strideSize: Positive integer specifying size a stride \
        from one patch to next in input image space in X and Y dimension.
        """

        self.setImgParamsXY(imgSize, patchSize, strideSize,
                            imgSize, patchSize, strideSize)

    @property
    def regularizationFactor(self):
        """Regularisation parameter/Lagrange multiplier lambda, which quantifies
        the contribution of L1-norm in the objective function; subtracted from
        the bias current after scaling in the LCA dynamical system implementation"""
        self._assertIsDefined('_regularizationFactor')
        return self._regularizationFactor

    @regularizationFactor.setter
    def regularizationFactor(self, val):
        self._regularizationFactor = val

    @property
    def biasScaling(self):
        """Scaling applied to the bias current"""
        self._assertIsDefined('_biasScaling')
        return self.tauU * 2**(6+self.wgtPrecision)

    # ToDo: What is this used for?
    @property
    def biasPrecision(self):
        """Hardware-specific bit-precision for the bias current;
        default value is set to 13-bits, including sign"""
        self._assertIsDefined('_biasPrecision')
        return self._biasPrecision

    @property
    def dictSize(self):
        """Number of atoms/elements in the overcomplete dictionary"""
        self._assertIsDefined('_dictSize')
        return self._dictSize

    @dictSize.setter
    def dictSize(self, val):
        self._validatePosInteger(val, 'dictSize')
        self._dictSize = val

    @property
    def numNeuronsPerGroup(self):
        self._assertIsDefined('_dictSize')
        return self._dictSize * 2

    @property
    def tauU(self):
        """Returns compartmentCurrentTimeConstant computed from
        compartmentCurrentDecay
        tauU = dt/decayU*2**12
        """
        return self.timePerStep/self.decayU*2**12

    @tauU.setter
    def tauU(self, val):
        """Sets compartmentCurrentDecay via compartmentCurrentTimeConstant:
        decayU = dt/tauU*2**12
        """
        self.decayU = int(self.timePerStep/val*2**12)

    @property
    def decayU(self):
        """ *Inverse* of time-constant for current decay in a neuron/compartment;
        smaller value => faster decay"""
        self._assertIsDefined('_decayU')
        return self._decayU

    @decayU.setter
    def decayU(self, val):
        self._validatePosInteger(val, 'decayU')
        assert val < 2**12, 'decayU must be < 2**12.'
        self._decayU = val

    @property
    def decayV(self):
        """ *Inverse* of time-constant for membrane voltage decay in a neuron/compartment;
        smaller value => faster decay"""
        self._assertIsDefined('_decayV')
        return self._decayV

    @decayV.setter
    def decayV(self, val):
        self._validatePosInteger(val, 'decayV')
        assert val < 2 ** 12, 'decayV must be < 2**12.'
        self._decayV = val

    @property
    def vth(self):
        """Threshold membrane voltage above which a neuron fires"""
        self._assertIsDefined('_vth')
        return self._vth

    @vth.setter
    def vth(self, val):
        self._validatePosInteger(val, 'vth')
        assert val < 2**23, 'vth must be < 2**23'
        self._vth = val

    @property
    def wgtExp(self):
        """Synaptic weights are scaled by 2^wgtExp when they are
        used in the hardware; see also, wgtPrecision"""
        self._assertIsDefined('_wgtExp')
        return self._wgtExp

    @wgtExp.setter
    def wgtExp(self, val):
        assert isinstance(val, int) and \
            -2**3 <= val < 2**3, 'wgtExp must be a signed 4 bit integer.'
        self._wgtExp = val

    @property
    def wgtPrecision(self):
        """Desired bit-precision for synaptic weights.
        Should satisfy wgtPrecision + abs(wgtExp) = 8"""
        assert self._wgtBits is not None, 'wgtPrecision undefined.'
        if self._wgtBits == 8:
            return 7
        else:
            return self._wgtBits

    @wgtPrecision.setter
    def wgtPrecision(self, val):
        self._validatePosInteger(val, 'wgtPrecision')
        assert val <= 8 and val != 7, 'wgtPrecision must be in [1, 2, 3, ' \
            '4, 5, 6, 8].'
        if val == 8:
            self._wgtBits = 7
        else:
            self._wgtBits = val

    @property
    def useMixedWeights(self):
        """Flag to toggle incorporating sign of weights in the weight value itself;
        when True, weights are signed integers. When False, weights are unsigned
        integers, and then there is a separate variable for their sign"""
        self._assertIsDefined('_useMixedWeights')
        return self._useMixedWeights

    @useMixedWeights.setter
    def useMixedWeights(self, val):
        assert isinstance(val, int) and \
            val in [0, 1], 'useMixedWeights must be 0 or 1.'
        self._useMixedWeights = val

    @property
    def useOverlapBasedWgtScaling(self):
        self._assertIsDefined('_useOverlapBasedWgtScaling')
        return self._useOverlapBasedWgtScaling

    @useOverlapBasedWgtScaling.setter
    def useOverlapBasedWgtScaling(self, val):
        assert isinstance(val, int) and \
            val in [0, 1], 'useOverlapBasedWgtScaling must be 0 or 1.'
        self._useOverlapBasedWgtScaling = val

    @property
    def maxConnectionDistance(self):
        self._assertIsDefined('_maxConnectionDistance')
        return self._maxConnectionDistance

    @maxConnectionDistance.setter
    def maxConnectionDistance(self, val):
        assert isinstance(val, int) and val >= 0, \
            'maxConnectionDistance must be >= 0.'
        self._maxConnectionDistance = val

    @property
    def enableSomaTrace(self):
        """Enables soma traces for activity monitoring"""
        self._assertIsDefined('_enableSomaTrace')
        return self._enableSomaTrace

    @enableSomaTrace.setter
    def enableSomaTrace(self, val):
        self._validatePosInteger(val, 'enableSomaTrace')
        assert val in (0, 1), 'enableSomaTrace must be 0 or 1.'
        self._enableSomaTrace = val

    @property
    def tEpochDend(self):
        """Legnth of dendritic epoch in number of algorithmic time-steps"""
        self._assertIsDefined('_tEpochDend')
        return self._tEpochDend

    @tEpochDend.setter
    def tEpochDend(self, val):
        self._validatePosInteger(val, 'tEpochDend')
        assert val < 2**6, 'tEpochDend must be < 2**6.'
        self._tEpochDend = val

    @property
    def somaTau(self):
        """Time constant associated with somatic processes; used to
        compute decay constant of somatic current 'a', etc."""
        self._assertIsDefined('_somaTau')
        return self._somaTau

    @somaTau.setter
    def somaTau(self, val):
        self._validatePosInteger(val, '_somaTau')
        self._somaTau = val

    @property
    def somaSpikeLevelInt(self):
        """Integer part of amount by which somatic current 'a' should
        increase, upon receiving a spike"""
        self._assertIsDefined('_somaSpikeLevelInt')
        return self._somaSpikeLevelInt

    @somaSpikeLevelInt.setter
    def somaSpikeLevelInt(self, val):
        self._validatePosInteger(val, 'somaSpikeLevelInt')
        assert val < 2**7, 'somaSpikeLevelInt must be < 2**7'
        self._somaSpikeLevelInt = val

    @property
    def somaSpikeLevelFrac(self):
        """Fractional part of amount by which somatic current 'a' should
        increase, upon receiving a spike"""
        self._assertIsDefined('_somaSpikeLevelFrac')
        return self._somaSpikeLevelFrac

    @somaSpikeLevelFrac.setter
    def somaSpikeLevelFrac(self, val):
        self._validatePosInteger(val, 'somaSpikeLevelFrac')
        assert val < 2**8, 'somaSpikeLevelFrac must be < 2**8'
        self._somaSpikeLevelFrac = val

    @property
    def somaDecay(self):
        """ *Inverse* of time-constant associated with decay of somatic
        current 'a'; internally computed as a function of somaTau"""
        self._assertIsDefined('_somaTau')
        tcg = TraceCfgGen()
        return tcg.calcDecay(self._somaTau)

    @property
    def somaExpTimeConst(self):
        self._assertIsDefined('_somaTau')
        tcg = TraceCfgGen()
        return tcg.calcExpTimeConst(self._somaTau)

    @property
    def somaRandomThreshold(self):
        self._assertIsDefined('_somaTau')
        tcg = TraceCfgGen()
        return tcg.calcRndThresh(self._somaTau)

    @property
    def timePerStep(self):
        """Physical (i.e., wall-clock) time elapsed per algorithmic time-step;
        used in rate-computation for reconstruction"""
        self._assertIsDefined('_timePerStep')
        return self._timePerStep

    @timePerStep.setter
    def timePerStep(self, val):
        assert val > 0, 'timePerStep must be a positive number'
        self._timePerStep = val

    @property
    def numEpochs(self):
        """Number of times LcaNet will be run, each run
        comprised of numStepsPerEpoch time-steps"""
        self._assertIsDefined('_numEpochs')
        return self._numEpochs

    @numEpochs.setter
    def numEpochs(self, val):
        self._validatePosInteger(val, 'numEpochs')
        self._numEpochs = val

    @property
    def numStepsPerEpoch(self):
        """Number of time steps of a single epoch. The total runtime is \
        numStepsPerEpoch*numEpochs. Determines the interval at which \
        somaTraces are reset by host to avoid saturation."""
        self._assertIsDefined('_numStepsPerEpoch')
        return self._numStepsPerEpoch

    @numStepsPerEpoch.setter
    def numStepsPerEpoch(self, val):
        """Number of time steps of a single epoch. The total runtime is \
        numStepsPerEpoch*numEpochs. Determines the interval at which \
        somaTraces are reset by host to avoid saturation."""
        self._validatePosInteger(val, 'numStepsPerEpoch')
        self._numStepsPerEpoch = val

    @property
    def probeInterval(self):
        """Interval at which somaTraces are read out by host."""
        self._assertIsDefined('_probeInterval')
        return self._probeInterval

    @probeInterval.setter
    def probeInterval(self, val):
        """Interval at which somaTraces are read out by host."""
        self._validatePosInteger(val, 'probeInterval')
        self._probeInterval = val

    @property
    def probeOffset(self):
        """Number of time-steps skipped (starting from 0) before any
        probing starts"""
        self._assertIsDefined('_probeOffset')
        return self._probeOffset

    @probeOffset.setter
    def probeOffset(self, val):
        self._validatePosInteger(val, 'probeOffset')
        self._probeOffset = val

    @property
    def enableSpikes(self):
        """Enables sending of spikes between neurons. If False, axons are
        disconnected."""
        self._assertIsDefined('_enableSpikes')
        return self._enableSpikes

    @enableSpikes.setter
    def enableSpikes(self, val):
        assert isinstance(val, bool), 'enableSpikes must be boolean.'
        self._enableSpikes = val

    @property
    def disableOffboardSpikes(self):
        """Enables sending of spikes between neurons. If False, axons are
        disconnected."""
        self._assertIsDefined('_enableSpikes')
        return self._disableOffboardSpikes

    @disableOffboardSpikes.setter
    def disableOffboardSpikes(self, val):
        assert isinstance(val, bool), 'enableSpikes must be boolean.'
        self._disableOffboardSpikes = val

    # Helper methods
    def _assertIsDefined(self, field):
        assert getattr(self, field) is not None, '%s undefined' % (field[1:])

    @staticmethod
    def _validatePosInteger(val, name):
        assert isinstance(val, int) and val >= 0, \
            '%s must be a positive integer.' % (name)

    @staticmethod
    def computeNumStrides(imgSize, patchSize, strideSize):
        """Computes the number of strides for given imgSize, patchSize and
        strideSize.
        Raises AssertionError if number of strides is not an integer.

        :param int imgSize: Image size in a dimension.
        :param int patchSize: Size of a patch in a dimension.
        :param int strideSize: Size of stride in a dimension.
        :return numStrides: Integer number of strides in a dimension.
        :rtype: int
        """
        if strideSize == 0:
            numStrides = 1
        else:
            numStrides = (imgSize - patchSize)/strideSize + 1
        assert numStrides % 1 == 0, 'numStrides is not integer.'
        return int(numStrides)


class CompileParams:
    """Parameter class for configuring compilation parameters for LcaNet."""

    def __init__(self):
        self.maxNumNeuronGroupsPerCore = 16
        self.maxNumNeuronsPerCore = 1024
        self.maxNumSynMemWordsPerCore = 2**14
        self._packingMode = 'tight'
        self._maxNumCoresPerChip = 128
        self._disableNeuronUpdates = False
        self._cxSpacingMode = 'interleaved'
        self._popMsgSize = 32
        self._useMultiChip = False
        self._partition = None
        self._optimizeCorePlacement = False

    # --------------------------------------------------------------------------
    # Helper methods
    def _assertIsDefined(self, field):
        assert getattr(self, field) is not None, '%s undefined' % (field[1:])

    # --------------------------------------------------------------------------
    # Getter/setter interface
    @property
    def maxNumCoresPerChip(self):
        """The maximum number of neurons allowed per neuroCore."""
        self._assertIsDefined('_maxNumCoresPerChip')
        return self._maxNumCoresPerChip

    @maxNumCoresPerChip.setter
    def maxNumCoresPerChip(self, val):
        assert isinstance(val, int), 'maxNumCoresPerChip must be a ' \
                                     'positive integer.'
        assert val > 0 and val <= 128, 'maxNumCoresPerChip must be with 0 ' \
            'and 128.'
        self._maxNumCoresPerChip = val

    @property
    def maxNumNeuronGroupsPerCore(self):
        """The maximum number of neuronGroups allowed per neuroCore."""
        self._assertIsDefined('_maxNumNeuronGroupsPerCore')
        return self._maxNumNeuronGroupsPerCore

    @maxNumNeuronGroupsPerCore.setter
    def maxNumNeuronGroupsPerCore(self, val):
        assert isinstance(val, int), 'maxNumNeuronGroupsPerCore must be a ' \
                                     'positive integer.'
        assert val > 0 and val <= 16, 'maxNumNeuronGroupsPerCore must be with ' \
                                      'and 16.'
        self._maxNumNeuronGroupsPerCore = val

    @property
    def maxNumNeuronsPerCore(self):
        """The maximum number of neurons allowed per neuroCore."""
        self._assertIsDefined('_maxNumNeuronsPerCore')
        return self._maxNumNeuronsPerCore

    @maxNumNeuronsPerCore.setter
    def maxNumNeuronsPerCore(self, val):
        assert isinstance(val, int), 'maxNumNeuronsPerCore must be a ' \
                                     'positive integer.'
        assert val > 0 and val <= 1024, 'maxNumNeuronsPerCore must be with 0 ' \
                                        'and 1024.'
        self._maxNumNeuronsPerCore = val

    @property
    def maxNumSynMemWordsPerCore(self):
        """The maximum number of synMemWords allowed per neuroCore."""
        self._assertIsDefined('_maxNumSynMemWordsPerCore')
        return self._maxNumSynMemWordsPerCore

    @maxNumSynMemWordsPerCore.setter
    def maxNumSynMemWordsPerCore(self, val):
        assert isinstance(val, int), 'maxNumSynMemWordsPerCore must be a ' \
                                     'positive integer.'
        assert val > 0 and val <= 2**14, 'maxNumSynMemWordsPerCore must be ' \
            'with 0 and 2**14'
        self._maxNumSynMemWordsPerCore = val

    @property
    def packingMode(self):
        """Returns packingMode of compiler.
        'tight' attempts to pack neurons on as few cores as possible.
        'distributed' attempts to distribute neurons on fixed number of cores"""
        return self._packingMode

    @property
    def disableNeuronUpdates(self):
        """Disables neuron updates by setting NeuroCore numUpdates parameter
        to 0."""
        return self._disableNeuronUpdates

    @disableNeuronUpdates.setter
    def disableNeuronUpdates(self, val):
        """Disables neuron updates by setting NeuroCore numUpdates parameter
                to 0."""
        assert isinstance(val, bool), 'val must be boolean.'
        self._disableNeuronUpdates = val

    @property
    def cxSpacingMode(self):
        """Specifies the mode how compartments are allocated of a
        neuronGroup are allocated within a neuroCore. Possible modes are:

        * **interleaved**: Compartments in a neuronGroup get interleaved by \
        maxNumNeuronGroupsPerCore.
        * **sequential**: Compartments in a neuronGroup get sequentially \
        allocated without interleaving.
        """
        self._assertIsDefined('_cxSpacingMode')
        return self._cxSpacingMode

    @cxSpacingMode.setter
    def cxSpacingMode(self, val):
        assert isinstance(val, str) and val in ('interleaved', 'sequential'), \
            "cxSpacingMode must be 'interleaved' or 'sequential' ."
        self._cxSpacingMode = val

    @property
    def popMsgSize(self):
        """The spike message type used for population mode/connection
        sharing."""
        self._assertIsDefined('_popMsgSize')
        return self._popMsgSize

    @popMsgSize.setter
    def popMsgSize(self, val):
        assert isinstance(val, int) and val in (16, 32), \
            'popMsgSize must be 16 or 32.'
        self._popMsgSize = val

    @property
    def axonCfgSpacing(self):
        """Returns the spacing between successive axons in axonCfg depending
        on 'popMsgSize' and 'useMultiChip'. """
        self._assertIsDefined('_popMsgSize')
        self._assertIsDefined('_useMultiChip')
        val = self._useMultiChip
        if self.popMsgSize == 16:
            val += 1
        elif self.popMsgSize == 32:
            val += 2

        return val

    @property
    def useMultiChip(self):
        """"Specifies whether LCA net spans multiple chips and thus requires
        usage of remote header for spike messages. This will impact the
        number of output neurons per core."""
        self._assertIsDefined('_useMultiChip')
        return self._useMultiChip

    @useMultiChip.setter
    def useMultiChip(self, val):
        assert isinstance(val, bool), 'useMultiChip must be boolen.'
        self._useMultiChip = val

    @property
    def partition(self):
        """Returns the partition on which to execute LCA net."""
        return self._partition

    @partition.setter
    def partition(self, val):
        assert val is None or isinstance(val, str) and val in ('pohoikibeach',
                                                               'nahuku32',
                                                               'nahuku32_inf',
                                                               'nahuku08',
                                                               'wm_inf'), \
            "partition must be a string specifying the name of the partition " \
            "on which to execute network."
        self._partition = val

    @property
    def optimizeCorePlacement(self):
        """Boolean to enable optimization of NeuroCore placement, based solely on
        network architecture (network activity is not currently taken into account)"""
        return self._optimizeCorePlacement

    @optimizeCorePlacement.setter
    def optimizeCorePlacement(self, val):
        assert val is None or isinstance(val, bool), "Set optimizeCorePlacement flag to True or False"
        self._optimizeCorePlacement = val


class Solution():
    """Represents the solution of a sparse coding problem:
    argmin_a |x - D'*a|**2 + lambda*|a|_1
    It is returned by LcaNet.solve() and contains the sparseCode,
    the reconstruction error, the objective and sparsity as a function of time.
    """

    def __init__(self, lcaNet, time, numSpikes, sparseCode, reconstruction,
                 error, objective, sparsity):
        self.time = time
        self.input = np.copy(lcaNet.input)
        self.numSpikes = numSpikes
        self.timePerStep = lcaNet.modelParams.timePerStep
        self.rates = sparseCode
        self.reconstruction = reconstruction
        self.finalReconstruction = np.copy(
            self.reconstruction[-1, :, :].reshape(lcaNet.input.shape))
        self.error = error
        self.objective = objective
        self.sparsity = sparsity
        self.axonFanout = lcaNet._axonFanout
        self.synFanout = lcaNet._synFanout

    def print(self):
        """Prints the reconstruction error, objective and sparsity at the
        last time step of the solution."""
        info('\nSparse coding solution at t=%d:' % (self.time[-1]))
        info('Error     = {:.2f}'.format(self.error[-1]))
        info('Objective = {:.2f}'.format(self.objective[-1]))
        info('Sparsity  = {:.2f}%'.format(self.sparsity[-1]))

    @property
    def instRates(self):
        instRates = np.zeros(self.numSpikes.shape)
        nonZeroIdx = np.where(np.sum(self.numSpikes, axis=0) > 0)[0]
        for i in nonZeroIdx:
            numSpikes = np.polyfit(self.time, self.numSpikes[:, i], deg=2)
            derivative = np.polyder(numSpikes)
            instRates[:, i] = np.maximum(np.polyval(derivative,
                                                    self.time)/self.timePerStep, 0)

        return instRates

    @staticmethod
    def makeColorbar(mappable):
        from mpl_toolkits.axes_grid1 import make_axes_locatable
        ax = mappable.axes
        fig = ax.figure
        divider = make_axes_locatable(ax)
        cax = divider.append_axes("right", size="5%", pad=0.05)
        return fig.colorbar(mappable, cax=cax)

    def plotConvergence(self):
        """Plots error, objective and sparsity."""

        plt.subplot(3, 1, 1)
        plt.plot(self.time, self.error)
        plt.title('Reconstruction error')

        plt.subplot(3, 1, 2)
        plt.plot(self.time, self.objective)
        plt.title('Objective')

        plt.subplot(3, 1, 3)
        plt.plot(self.time, self.sparsity)
        plt.xlabel('Time (in # of time-steps)')
        plt.title('Activity (%)')

    def plotReconstruction(self):
        """Plots original image and reconstruction image."""

        ax1 = plt.subplot(1, 2, 1)
        ax1.xaxis.set_visible(False)
        ax1.yaxis.set_visible(False)
        img1 = plt.imshow(self.input, cmap='gray', aspect='equal')
        plt.title('Original')
        self.makeColorbar(img1)

        ax2 = plt.subplot(1, 2, 2)
        ax2.xaxis.set_visible(False)
        ax2.yaxis.set_visible(False)
        img2 = plt.imshow(self.finalReconstruction,
                          cmap='gray', aspect='equal')
        plt.title('Reconstruction')
        self.makeColorbar(img2)

    def plot(self, figId1=10, figId2=11, figsize=(10, 10)):
        """Plots temporal evolution of reconstruction error, objective
        and sparsity of the LASSO problem."""

        fig1 = plt.figure(figId1, figsize=figsize)
        self.plotConvergence()

        plt.figure(figId2, figsize=figsize)
        self.plotReconstruction()

        fig1.tight_layout()
        plt.show()


class VerbosityLevel(IntEnum):
    """Can be used to control verbosity level of status log statements in
    LcaNet (Is currently not used widely in LcaNet)."""
    LOW = 0
    MID = 1
    HIGH = 2


class LcaNet:
    """Represents a neural network that solves a sparse coding problem using\
    a spiking version of a locally competitive algorithm (LCA) according to:\
    https://arxiv.org/pdf/1705.05475.pdf
    The sparse coding problem can be summarized as:
    Find a sparse code 'a' that minimizes |x - D'*a|**2 + lambda*|a|
    Where x is the input, D is a dictionary and lambda is a sparsity
    parameter that controls the impact of the regularization term |a|.

    The LcaNet module provides an interface to set up a single layer network\
    of a given size, configure the input and a dictionary, execute the\
    network, probe state and visualize the network structure and output.

    Input to the network is provided in form of neuronal bias currents that
    are computed from the user-provided dictionary matrix and input.
    Furthermore, neurons are mutually connected with inhibitory weights based
    on th similarity of the dictionary elements that these neurons correspond
    to. As a result, neurons initially start to spike in proportion to their
    driving bias current but at the same time, neurons representing similar
    features (i.e. having similar receptive fields) inhibit each other and
    compete for representing their current input.

    Neurons in the network are organized in groups. Each neuronGroup,
    represents a certain sized image patch (a 2D location in the input space).
    The input space is of size (imgSizeY x imgSizeX) and each patch is of size
    (patchSizeY x patchSizeX) which must be smaller than the input space.
    These patches stride across the input space and are spaced by
    strideSizeX/Y.
    Neurons within a group are driven by a bias current b that is
    proportional to dictionary D and the input x_p within the patch:
    b = D * x_p.
    The user-provided dictionary is of size
    (numNeuronsPerGroup x patchSizeX*patchSizeY) and thus x_p is of size
    (patchSizeX*patchSizeY x 1).
    To compute the inhibitory weights between neuronGroups, neuronGroups
    corresponding to fully and partially overlapping patches have to be
    distinguished.
    For fully overlapping patches, the recurrent self-inhibitory weight
    matrix of neuronGroup[i] is essentially:
    W_ii = -D_i * D_i'
    where D_i is the user-provided dictionary.
    For partially overlapping patches, the weight matrix between
    neuronGroups[j] and neuronGroup[i] is:
    W_ij = -D_i * D_j'
    Here, D_i and D_j are extended matrices created from the original dictionary
    D of size (numNeuronsPerGroup x overlapSpaceX*overlapSpaceY).
    The overlapSpace represents the smallest possible bounding box enclosing the
    patches of neuronGroups i and j with the row vectors of D being inserted in
    the locations of D_i and D_j corresponding to the location of the individual
    patches within the overlap space.
    Thereby the dot-product (i.e. similarity) between D_i and D_j takes the
    partial spatial overlap between the involved patches into account.

    In order to setup and configure a network, use the LcaNet() constructor and\
    provide an instance of ModelParams and CompileParams.\
    ModelParams contains functionally relevant model parameters such as\
    network size, patchSize, stride, sparsity parameter, weight precision, etc.\
    CompileParams contains parameters relevant for compilation. Using\
    defaults is usually sufficient.\
    After constructing the LcaNet module, the network can be generated via
    LcaNet.generateNetwork(). This will generate the network according to
    user settings and allocate the required resources.

    Once the network is generated, new dictionaries and inputs can be passed
    to the LcaNet module at any time via setDictionary() and setInput()
    (ideally in that order if a new dictionary is provided).

    In order to debug network activity of read out the solution of the sparse
    sparse coding problem, probes can be configured using
    LcaNet.createProbes() and network execution can be launched with
    LcaNet.run(). LcaNet.solve() can also be used as a short-hand alias that
    probe the required states, run the network and return the result.
    """

    def __init__(self, modelParams, compileParams):
        """Initializes LCA network model using ModelParams and CompileParams
        classes.
        :param ModelParams modelParams: An instance of ModelParams to
        configure model-related parameters.
        :param CompileParams compileParams: An instance of CompileParams to
        configure compilation-related parameters.
        """

        self.modelParams = modelParams
        self.compileParams = compileParams
        self.board = None
        self._netIsGenerated = False
        self._dictIsSet = False
        self._inputIsSet = False
        self._dictionary = None
        self._weights = OrderedDict()
        self._input = None
        self._biases = OrderedDict()
        self.neuronGroups = OrderedDict()
        self.connectivityGroups = OrderedDict()
        self.verbosity = VerbosityLevel.LOW
        self._overlapMasks = OrderedDict()
        self._connGroupMap = OrderedDict()
        self._cgToWeightsMap = OrderedDict()
        self._resourceEstimator = None
        self._activityProbes = None
        self._numEpochs = 0

    # --------------------------------------------------------------------------
    # Setter and getters
    @property
    def logBoardStatus(self):
        # return self.board.nxDriver.logStatus
        pass

    @logBoardStatus.setter
    def logBoardStatus(self, val):
        # self.board.nxDriver.logStatus = val
        pass

    @property
    def numPatches(self):
        """Returns number of patches."""
        return len(self.patches)

    @property
    def numNeurons(self):
        """Returns total number of neurons in LcaNet."""
        return self.modelParams.numNeuronsPerGroup * self.numPatches

    @property
    def numNeuronGroups(self):
        """Returns the number of populations, i.e., neuron groups"""
        return len(self.neuronGroups)

    @property
    def numConnectivityGroups(self):
        """Returns the number of connectivityGroups."""
        return len(self.connectivityGroups)

    @property
    def numNeuroCores(self):
        """Returns number of neuroCores."""
        return len(self.neuroCores)

    @property
    def resourceRequirements(self):
        """Returns the resourceRequirements object once network has been
        generated."""
        assert self._resourceEstimator is not None, \
            "ResourceRequirements haven't been computed yet."
        return self._resourceEstimator

    @property
    def dictionary(self):
        """Returns current dictionary as a numpy.array."""

        assert self._dictionary is not None, 'No dictionary defined yet.'
        return self._dictionary

    @dictionary.setter
    def dictionary(self, val):
        """Sets a new dictionary."""

        assert isinstance(val, np.ndarray), 'dictionary must be a (dictSize x ' \
            'imgSize) numpy array.'
        dictShape = (self.modelParams.dictSize,
                     self.modelParams.patchSizeY*self.modelParams.patchSizeX)
        valShape = np.shape(val)
        assert valShape == dictShape, \
            'dictionary array has invalid shape. Must be (dictSize x ' \
            'patchSizeY*patchSizeX) = (%d,%d) but is (%d,%d)' % \
            (dictShape[0], dictShape[1], valShape[0], valShape[1])
        self._dictionary = val

    @property
    def input(self):
        """Returns current image as numpy array."""

        assert self._input is not None, "No input defined yet."
        return self._input

    @input.setter
    def input(self, val):
        """Sets a new input."""

        assert isinstance(val, np.ndarray), \
            'input must be a (imgSizeY x imgSizeX) numpy.array'
        inputShape = np.shape(val)
        assert inputShape == (self.modelParams.imgSizeY,
                              self.modelParams.imgSizeX), \
            'Invalid input shape. Must be (imgSizeY x imgSizeX) numpy.array.'
        self._input = val

    @property
    def verbosity(self):
        return self._verbosity

    @verbosity.setter
    def verbosity(self, val):
        assert isinstance(val, VerbosityLevel)
        self._verbosity = val

    # --------------------------------------------------------------------------
    # Helper
    def _computePatchCoverage(self):
        """Adds up all the location masks of each Patch and returns a numpy
        array where each element measures how many patches cover each pixel."""

        mp = self.modelParams
        patchCoverage = np.zeros((mp.imgSizeY, mp.imgSizeX))
        for p in self.patches.values():
            mask = p.getMask(mp.imgSizeX, mp.imgSizeY)
            patchCoverage += mask
        return patchCoverage

    def _computeLocations(self, imgSize, patchSize, strideSize):
        """Computes locations of patches."""
        if strideSize == 0:
            # ToDo: Nonsene shoult be strideSize==patchSize
            locations = [0]
        else:
            locations = range(0, imgSize - patchSize + 1, strideSize)
        return locations

    def _getOverlapLocations(self, x, y):
        """Computes the (x_i, y_i) patch locations that overlap with the patch
        at (x, y) given imageSize, patchSize, strideSize and
        maxConnectionDistance."""

        def getLocRange(z, patchSize, strideSize, maxConnDist):
            connDist = int(min(patchSize/strideSize - 1, maxConnDist))
            maxDist = connDist * strideSize
            return range(z-maxDist, z+maxDist+1, strideSize)

        mp = self.modelParams
        locations = []
        # Generate list of all potential overlapping patch location coordinates
        if mp.strideSizeX > 0:
            xLocs = getLocRange(x, mp.patchSizeX, mp.strideSizeX,
                                mp.maxConnectionDistance)
            #xLocs = range(x - mp.patchSizeX + mp.strideSizeX,
            #              x + mp.patchSizeX, mp.strideSizeX)
        else:
            xLocs = [x]
        if mp.strideSizeY > 0:
            yLocs = getLocRange(y, mp.patchSizeY, mp.strideSizeY,
                                mp.maxConnectionDistance)
            #yLocs = range(y - mp.patchSizeY + mp.strideSizeY,
            #              y + mp.patchSizeY, mp.strideSizeY)
        else:
            yLocs = [y]

        # Create list of all overlapping patch locations
        for xi in xLocs:
            for yi in yLocs:
                if xi >= 0 and (xi+mp.patchSizeX) <= mp.imgSizeX \
                        and yi >= 0 and (yi+mp.patchSizeY) <= mp.imgSizeY:
                    locations.append((xi, yi))

        return locations

    def _generatePatches(self):
        """Generates patches for each patch location in the image space."""

        mp = self.modelParams
        locationsX = self._computeLocations(mp.imgSizeX, mp.patchSizeX,
                                            mp.strideSizeX)
        locationsY = self._computeLocations(mp.imgSizeY, mp.patchSizeY,
                                            mp.strideSizeY)
        self.patches = OrderedDict()
        ltpMap = OrderedDict()
        pId = 0
        for x in locationsX:
            for y in locationsY:
                p = Patch(pId, x, y, mp.patchSizeX, mp.patchSizeY)
                self.patches[pId] = p
                pId += 1
                ltpMap[(x, y)] = p

        self._patchToPatchMap = OrderedDict()
        for p in self.patches.values():
            locs = self._getOverlapLocations(p.x0, p.y0)
            self._patchToPatchMap[p] = [ltpMap[loc] for loc in locs]

    def _generateGroupsOld(self):
        """Generate neuronGroups and connectivityGroups. Only neuronGroups
        whose receptive fields overlap within the input space get connected
        via a connectivityGroup. All pairs of neuronGroups whose receptive
        fields overlap in the same way irrespective of the absolute location
        of a neuronGroup's patch in the input space share the same
        connectivityGroup.
        """

        # Check that there are nonzero patches
        assert self.numPatches > 0, "Cannot create neuronGroups or " \
                                    "connectivityGroups without patches."
        # Initialize neuronGroups
        i = 0
        for pId in self.patches:
            # Assign neuronIds to neuronGroup
            nIds = range(i*self.modelParams.numNeuronsPerGroup,
                         (i+1)*self.modelParams.numNeuronsPerGroup)
            i += 1
            # Create and store neuronGroup
            ng = NeuronGroup(pId, nIds)
            self.neuronGroups[pId] = ng

        # Connect neuronGroups via connectivityGroups
        mp = self.modelParams
        for i, ng1 in self.neuronGroups.items():
            p1 = self.patches[i]
            for j, ng2 in self.neuronGroups.items():
                p2 = self.patches[j]
                # Check if patches associated with neuronGroups overlap
                mask, maskKey = p1.getOverlapHash(
                    p2, mp.imgSizeX, mp.imgSizeY)
                # If there's overlap, connect neuronGroups
                if mask is not None:
                    if maskKey in self.connectivityGroups.keys():
                        # Use existing connectivityGroup for this kind of
                        # overlap
                        cg = self.connectivityGroups[maskKey]
                    else:
                        # Create new connectivityGroup for this kind of overlap
                        cg = ConnectivityGroup(self.numConnectivityGroups)
                        self.connectivityGroups[maskKey] = cg
                        self._overlapMasks[maskKey] = mask
                    # Connect neuronGroups by connectivityGroup
                    cg.connect(ng1, ng2)
                    self._connGroupMap[(ng1, ng2)] = cg

    def _generateGroups(self):
        """Generate neuronGroups and connectivityGroups. Only neuronGroups
        whose receptive fields overlap within the input space get connected
        via a connectivityGroup. All pairs of neuronGroups whose receptive
        fields overlap in the same way irrespective of the absolute location
        of a neuronGroup's patch in the input space share the same
        connectivityGroup.
        """

        # Check that there are nonzero patches
        assert self.numPatches > 0, "Cannot create neuronGroups or " \
                                    "connectivityGroups without patches."
        # Initialize neuronGroups
        i = 0
        ptnMap = OrderedDict()
        for pId, p in self.patches.items():
            # Assign neuronIds to neuronGroup
            nIds = range(i*self.modelParams.numNeuronsPerGroup,
                         (i+1)*self.modelParams.numNeuronsPerGroup)
            i += 1
            # Create and store neuronGroup
            ng = NeuronGroup(pId, nIds)
            self.neuronGroups[pId] = ng
            ptnMap[p] = ng

        # Connect neuronGroups via connectivityGroups
        mp = self.modelParams
        for i, ng1 in self.neuronGroups.items():
            p1 = self.patches[i]
            for p2 in self._patchToPatchMap[p1]:
                ng2 = ptnMap[p2]
                # ToDo: This check is now unnecessary
                # Check if patches associated with neuronGroups overlap
                mask, maskKey = p1.getOverlapHash(
                    p2, mp.imgSizeX, mp.imgSizeY)
                # If there's overlap, connect neuronGroups
                if mask is not None:
                    if maskKey in self.connectivityGroups.keys():
                        # Use existing connectivityGroup for this kind of
                        # overlap
                        cg = self.connectivityGroups[maskKey]
                    else:
                        # Create new connectivityGroup for this kind of overlap
                        cg = ConnectivityGroup(self.numConnectivityGroups)
                        self.connectivityGroups[maskKey] = cg
                        self._overlapMasks[maskKey] = mask
                    # Connect neuronGroups by connectivityGroup
                    cg.connect(ng1, ng2)
                    self._connGroupMap[(ng1, ng2)] = cg
                else:
                    raise ValueError('Illegal patch overlap.')

    def _computeResourceRequirements(self):
        """Computes resourceRequirements of this LCA network using
        the LcaResourceRequirements class"""

        # Find maximum number of connectivityGroups per neuronGroup
        numSynGroupsPerCore = 0
        for ngId, ng in self.neuronGroups.items():
            if ng.numInputConnections > numSynGroupsPerCore:
                numSynGroupsPerCore = ng.numInputConnections

        mp = self.modelParams
        cp  = self.compileParams
        # Compute resource requirements
        self._resourceEstimator = ResourceEstimator(
            mp.wgtPrecision,
            cp.maxNumNeuronGroupsPerCore,
            cp.maxNumNeuronsPerCore,
            cp.maxNumSynMemWordsPerCore,
            2**12,
            cp.useMultiChip,
            cp.popMsgSize,
            mp.maxConnectionDistance)

        self._resourceEstimator.computeXY(
            mp.numNeuronsPerGroup,
            mp.imgSizeX, mp.patchSizeX, mp.strideSizeX,
            mp.imgSizeY, mp.patchSizeY, mp.strideSizeY)

    def _computeCxSpacing(self, coreId):
        """Returns spacing of compartments in index space of the compartments
        in a neuronGroup"""

        rr = self.resourceRequirements
        if self.compileParams.cxSpacingMode == 'interleaved':
            if coreId < math.floor(rr.numNeuronGroups/rr.numNeuronGroupsPerCore) \
                    * rr.numSlices:
                return rr.numNeuronGroupsPerCore
            else:
                return rr.numNeuronGroups % rr.numNeuronGroupsPerCore
        elif self.compileParams.cxSpacingMode == 'sequential':
            return 1
        else:
            raise ValueError

    def _initializeNeuroCores(self):
        """Initializes a set of empty neuroCores."""

        # Initialize neuroCores.
        self.neuroCores = OrderedDict()
        for cId in range(0, self.resourceRequirements.numCores):
            self.neuroCores[cId] = NeuroCore(cId, self._computeCxSpacing(cId))

    def _initializeNeuronGroupSlices(self):
        """Initializes NeuronGroups and NeuronGroupSlices.
        NeuronGroupSlices are distributed across different neuroCores. As a
        result each, neuroCore contains the i-th neuronGroupSlices of n
        different neuronGroups that share the same input connectivityGroups.
        """
        # ToDo: Currently, neuronGroups are distributed across core
        # irrespective of how many input connectivityGroups they share.

        # Initialize temporary variables
        numSlices = self.resourceRequirements.numSlices
        firstCoreId = -numSlices
        maxNumNeuronGroupsPerCore = self.resourceRequirements.numNeuronGroupsPerCore
        numNeuronGroupsPerCore = maxNumNeuronGroupsPerCore
        neuroCores = None
        # Slice neuronGroups into neuronGroupsSlices
        for ngId, ng in self.neuronGroups.items():
            if numNeuronGroupsPerCore == maxNumNeuronGroupsPerCore:
                # If capacity of neuroCore is exceeded advance to new set of
                #  neuroCores
                firstCoreId += numSlices
                numNeuronGroupsPerCore = 0
                coreIds = range(firstCoreId, firstCoreId+numSlices)
                neuroCores = [self.neuroCores[cId] for cId in coreIds]
            # Slice neuronGroups across multiple cores
            ng.slice(neuroCores)
            numNeuronGroupsPerCore += 1

    def _allocateResources(self):
        """Allocates synapses, input axons, compartments and output axons
        for all neuronGroups."""

        numChips = int(math.ceil(
            self.resourceRequirements.numCores/self.compileParams.maxNumCoresPerChip))
        assert self.compileParams.useMultiChip or numChips == 1, \
            'numCores=%d, only single chip networks are supported when ' \
            'CompileParams.useMultiChip==False.' % (
                self.resourceRequirements.numCores)

        for ng in self.neuronGroups.values():
            for ngs in ng.neuronGroupSlices:
                core = ngs.neuroCore
                relNIds = ngs.getRelativeNeuronIds()

                # Allocate synapses
                for cg in ng.inputConnGroups.values():
                    # Get relativeAxonIds/relativeNeuronIds to synIds for
                    # connectivityGroup
                    if core.hasConnectivityGroup(cg, relNIds):
                        synIdMap = core.getAxonToSynIdMap(cg)
                    else:
                        # Allocate new synIds if this is the first time
                        # this connectivityGroup gets assigned to core.
                        synIdMap = core.allocateSynapses(
                            cg,
                            self.modelParams.numNeuronsPerGroup,
                            relNIds)
                    ngs.addConnGrpToAxonToSynIdMap(cg, synIdMap)

                # Allocate input axons
                for cg in ng.inputConnGroups.values():
                    popInAxonId = core.allocateInputAxons(1)
                    ngs.addConnGrpToInputAxonIdMap(cg, popInAxonId)

                # Allocate neurons
                cxIds = core.allocateCompartments(ngs.numNeurons)
                ngs.addCompartmentIds(cxIds)

                # Allocate output axons
                for cg in ng.outputConnGroups.values():
                    for toNg in cg.getToNeuronGroups(ng):
                        outAxonIds = core.allocateOutputAxons(
                            toNg.numSlices,
                            self.compileParams.axonCfgSpacing)
                        ngs.addConnGrpToOutputAxonIdMap(cg, toNg, outAxonIds)

    def _partitionNetwork(self):
        """Partitions LCA network by distributing all neuronGroups across
        neuroCores.
        """

        self._initializeNeuroCores()
        self._initializeNeuronGroupSlices()
        self._allocateResources()

    def _computeNumSynPerCorePerChip(self, nChips, nCores):
        """Calculate number of synapses per core per chip"""
        # Compute number of synapses to allocate on each N2Chip and on each N2Core
        numSynapses = [nc.numSynapses for nc in self.neuroCores.values()]
        i = 0
        nSynChips = []
        for chipId in range(0, nChips):
            numSynCores = []
            for coreId in range(0, nCores[chipId]):
                numSynCores.append(numSynapses[i])
                i += 1
            nSynChips.append(numSynCores)

        return nSynChips

    def _calcChipsCoresSynapses(self):
        """Create an instance of N2Board with requisite number of chips"""

        # Compute number of required N2Chips and N2Cores on each N2Chip
        numCoresPerChip = self.compileParams.maxNumCoresPerChip
        rr = self.resourceRequirements
        numChips = int(math.ceil(rr.numCores/numCoresPerChip))
        numCoresOnLastChip = (rr.numCores-1) % numCoresPerChip + 1
        numCores = [numCoresPerChip] * (numChips - 1)
        numCores.append(numCoresOnLastChip)
        numSynChips = self._computeNumSynPerCorePerChip(numChips, numCores)
        # Return number of chips, cores, synpases
        return numChips, numCores, numSynChips

    def _generateNetworkGraph(self):
        """Generate a graph with NeuronGroup vertices and
        ConnectivityGroup edges"""
        netGraph = ntx.MultiDiGraph()
        for cg in self.connectivityGroups.values():
            for ng1, ng2List in cg.forwardConnections.items():
                for ngs1 in ng1.neuronGroupSlices:
                    netGraph.add_node(ngs1)
                    for ng2 in ng2List:
                        for ngs2 in ng2.neuronGroupSlices:
                            netGraph.add_node(ngs2)
                            netGraph.add_edge(ngs1, ngs2, key=str(cg.id))
            for ng1, ng2List in cg.backwardConnections.items():
                for ngs1 in ng1.neuronGroupSlices:
                    netGraph.add_node(ngs1)
                    for ng2 in ng2List:
                        for ngs2 in ng2.neuronGroupSlices:
                            netGraph.add_node(ngs2)
                            netGraph.add_edge(ngs1, ngs2, key=str(cg.id))
        self._ngsGraph = netGraph

    def _generateNeuroCoreGraph(self):
        """Generate a graph with logical NeuroCore objects as
        vertices and ConnectivityGroups connecting NeuronGroupSlices
        on the NeuroCores as edges"""
        netGraph = ntx.OrderedMultiDiGraph()
        for cg in self.connectivityGroups.values():
            for ng1, ng2List in cg.forwardConnections.items():
                for ngs1 in ng1.neuronGroupSlices:
                    netGraph.add_node(ngs1.neuroCore, label=ngs1.neuroCore.id)
                    for ng2 in ng2List:
                        for ngs2 in ng2.neuronGroupSlices:
                            netGraph.add_node(ngs2.neuroCore, label=ngs2.neuroCore.id)
                            if netGraph.has_edge(ngs1.neuroCore, ngs2.neuroCore, key=str(cg.id)):
                                netGraph[ngs1.neuroCore][ngs2.neuroCore][str(cg.id)]['weight'] += 1
                            else:
                                netGraph.add_edge(ngs1.neuroCore, ngs2.neuroCore, key=str(cg.id), weight=1)
            # for ng1, ng2List in cg.backwardConnections.items():
            #     for ngs1 in ng1.neuronGroupSlices:
            #         netGraph.add_node(ngs1.neuroCore, label=ngs1.neuroCore.id)
            #         for ng2 in ng2List:
            #             for ngs2 in ng2.neuronGroupSlices:
            #                 netGraph.add_node(ngs2.neuroCore, label=ngs2.neuroCore.id)
            #                 if netGraph.has_edge(ngs1.neuroCore, ngs2.neuroCore, key=str(cg.id)):
            #                     netGraph[ngs1.neuroCore][ngs2.neuroCore][str(cg.id)]['weight'] += 1
            #                 else:
            #                     netGraph.add_edge(ngs1.neuroCore, ngs2.neuroCore, key=str(cg.id), weight=1)

        self._ncGraph = netGraph

    def _optimizeNetByClusteringCores(self, nChips, nCores):
        import nxsdk_modules.lca.scratch.network_optimizer as netOpt
        # The neurocores connected tightly with each other form clusters.
        # The flat node-list below is sorted according to cluster index.
        _, _, _, flatSortedNodeList = \
            netOpt.clusterNodes(self._ncGraph, numCoresPerChip=nCores[0])
        # Re-assigning the neuroCores list to the sorted list.
        # This way, when _buildNetwork assigns physical cores serially
        # through the list, the members of a cluster get assigned
        # consecutive physical neuro-cores.
        for j, neuroCore in enumerate(flatSortedNodeList):
            self.neuroCores[j] = neuroCore
        # Associate N2Cores to NeuroCore group
        self._seriallyAssociateNeuroCores(nChips, nCores)

    def _optimizeNetViaAdjMatrixOfCoreGraph(self, nChips, nCores):
        import nxsdk_modules.lca.scratch.network_optimizer as netOpt
        chipIdToFirstCoreIdMap = netOpt.partitionGraphUsingAdjMat_LCA(self._ncGraph,
                                                                      maxNumNodesPerBucket=nCores[0])
        chipIds = list(chipIdToFirstCoreIdMap.keys())
        totalNumCores = self.resourceRequirements.numCores
        for j, chipId in enumerate(chipIds):
            if j < len(chipIds) - 1:
                self.chipToCoreMap[chipId] = list(range(chipIdToFirstCoreIdMap[chipId],
                                                        chipIdToFirstCoreIdMap[chipId + 1]))
            if j == len(chipIds) - 1:
                self.chipToCoreMap[chipId] = list(range(chipIdToFirstCoreIdMap[chipId],
                                                        totalNumCores))

    def _seriallyAssociateNeuroCores(self, nChips, nCores):
        cumulativeNumCores = np.cumsum(np.array(nCores))
        cumulativeNumCores = np.append(0, cumulativeNumCores)
        # Serially associating logical neuroCores with physical n2Cores without any optimisation
        for chipId in range(0, nChips):
            self.chipToCoreMap[chipId] = list(range(cumulativeNumCores[chipId],
                                                    cumulativeNumCores[chipId+1]))

    def _associateNetwork(self):
        """Optimize network by reorganizing neuroCore placement, based on interconnectivity.
        A highly connected pair of cores should end up physically close on the chip to avoid
        communication bottlenecks in spiking phase."""

        numChips, numCores, numSynChips = self._calcChipsCoresSynapses()
        self.chipToCoreMap = OrderedDict()

        if self.compileParams.optimizeCorePlacement:
            info('Optimizing LcaNet core placement')
            self._generateNeuroCoreGraph()
            self._optimizeNetViaAdjMatrixOfCoreGraph(nChips=numChips, nCores=numCores)
        else:
            self._seriallyAssociateNeuroCores(numChips, numCores)

    def _startDriver(self):
        """Starts NxDriver when specific partition is requested."""

        partition = self.compileParams.partition
        if partition is not None:
            os.environ['PARTITION'] = partition
            self.board.start()
            os.unsetenv('PARTITION')

    def _buildSharedRegisters(self):
        """Initializes all N2Core's shared registers."""

        mp = self.modelParams
        rr = self.resourceRequirements
        # Determine minimum number of synapseFmt.idxBits based on
        # numNeuronsPerSlice
        if rr.numNeuronsPerSlice <= 2 ** 6:
            idxBits = 1
        elif rr.numNeuronsPerSlice <= 2 ** 7:
            idxBits = 2
        elif rr.numNeuronsPerSlice <= 2 ** 8:
            idxBits = 3
        elif rr.numNeuronsPerSlice <= 2 ** 9:
            idxBits = 4
        elif rr.numNeuronsPerSlice <= 2 ** 10:
            idxBits = 5
        else:
            raise ValueError('numNeuronsPerSlice is out of range.')
        # Determine synapseFmt.fanoutType
        if mp.useMixedWeights:
            # Allow excitatory and inhibitory weights
            fanoutType = 1
        else:
            # Allow inhibitory weights only
            fanoutType = 3
        # Determine number of dendriteAccum delayBits
        # delayBits = int(math.log2(2**13/rr.maxNumNeuronsPerCore))
        delayBits = 3
        # Determine someTraceCfg
        somaDecay = mp.somaDecay
        somaExpTimeConst = mp.somaExpTimeConst
        somaRndThresh = mp.somaRandomThreshold

        # Initialize all N2Core's shared registers
        for nc in self.neuroCores.values():
            # Specify how many compartment groups to update per core
            if self.compileParams.disableNeuronUpdates:
                nc.n2Core.numUpdates.numUpdates = 0
            else:
                nc.n2Core.numUpdates.numUpdates = int(nc.numCompartments/4+1)
            # Initialize cxProfileCfg
            nc.n2Core.cxProfileCfg[0].decayU = mp.decayU
            nc.n2Core.cxProfileCfg[0].decayV = mp.decayV
            nc.n2Core.cxProfileCfg[0].bapAction = 1
            nc.n2Core.cxProfileCfg[0].refractDelay = 1
            # Initialize vthProfileCfg
            if mp.enableSomaTrace:
                nc.n2Core.vthProfileCfg[0].dynamicCfg.configure(
                    enableHomeostasis=1,
                    beta=0,
                    aMin=0,
                    aMax=127)
            else:
                nc.n2Core.vthProfileCfg[0].staticCfgCfg.configure(
                    enableHomeostasis=0,
                    useSomaVth=0,
                    vth=self.modelParams.vth)

            # Initialize dendriteAccumCfg
            nc.n2Core.dendriteAccumCfg[0].delayBits = delayBits
            # Initialize dendriteTimeState
            if mp.enableSomaTrace:
                nc.n2Core.dendriteTimeState[0].tepoch = mp.tEpochDend
            else:
                nc.n2Core.dendriteTimeState[0].tepoch = 0
            # Initialize timeState
            nc.n2Core.timeState[0].tepoch = 0
            # Initialize sharedCfg
            nc.n2Core.dendriteSharedCfg[0].disableInhibited = 0
            nc.n2Core.dendriteSharedCfg[0].negVmLimit = 15+4
            nc.n2Core.dendriteSharedCfg[0].posVmLimit = 7
            nc.n2Core.dendriteSharedCfg[0].dmOffset = 0
            nc.n2Core.dendriteSharedCfg[0].dsOffset = 1
            # Initialize somaTraceCfg
            nc.n2Core.somaTraceCfg[0].spikeLevelInt = mp.somaSpikeLevelInt
            nc.n2Core.somaTraceCfg[0].spikeLevelFrac = mp.somaSpikeLevelFrac
            nc.n2Core.somaTraceCfg[0].rthMultiplier = somaExpTimeConst
            nc.n2Core.somaTraceCfg[0].randomThreshold = somaRndThresh
            nc.n2Core.somaTraceCfg[0].decay0 = somaDecay[0]
            nc.n2Core.somaTraceCfg[0].decay1 = somaDecay[1]
            nc.n2Core.somaTraceCfg[0].decay2 = somaDecay[2]
            nc.n2Core.somaTraceCfg[0].decay3 = somaDecay[3]
            nc.n2Core.somaTraceCfg[0].decay4 = somaDecay[4]
            nc.n2Core.somaTraceCfg[0].decay5 = somaDecay[5]

            # Initialize synapseFmt for synEntries with 60 synapses
            nc.n2Core.synapseFmt[1].wgtExp = mp.wgtExp
            nc.n2Core.synapseFmt[1].wgtBits = mp._wgtBits
            nc.n2Core.synapseFmt[1].numSynapses = 60
            nc.n2Core.synapseFmt[1].cIdxOffset = 0
            nc.n2Core.synapseFmt[1].cIdxMult = nc.cxSpacing - 1
            nc.n2Core.synapseFmt[1].idxBits = idxBits
            nc.n2Core.synapseFmt[1].fanoutType = fanoutType
            nc.n2Core.synapseFmt[1].compression = 3
            # Initialize synapseFmt for synEntries with remaining synapses
            nc.n2Core.synapseFmt[2].wgtExp = mp.wgtExp
            nc.n2Core.synapseFmt[2].wgtBits = mp._wgtBits
            nc.n2Core.synapseFmt[2].numSynapses = 63
            nc.n2Core.synapseFmt[2].cIdxOffset = 0
            nc.n2Core.synapseFmt[2].cIdxMult = nc.cxSpacing - 1
            nc.n2Core.synapseFmt[2].idxBits = idxBits
            nc.n2Core.synapseFmt[2].fanoutType = fanoutType
            nc.n2Core.synapseFmt[2].compression = 3

    def _buildDiscreteRegisters(self):
        """Initializes all N2Core's discrete registers."""

        useMultiChip = self.compileParams.useMultiChip
        chipAddrMap = None
        if self.compileParams.partition == 'pohoikibeach':
            chipAddrMap = (8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                           20, 21, 22, 23, 136, 137, 138, 139, 140, 141,
                           142, 143, 144, 145, 146, 147, 148, 149, 150,
                           151, 264, 265, 266, 267, 268, 269, 270, 271,
                           272, 273, 274, 275, 276, 277, 278, 279, 392,
                           393, 394, 395, 396, 397, 398, 399, 400, 401,
                           402, 403, 404, 405, 406, 407)
        elif self.compileParams.partition in ('nahuku32', 'nahuku32_inf'):
            chipAddrMap = (8, 9, 10, 11, 12, 13, 14, 15, 136, 137, 138, 139,
                           140, 141, 142, 143, 264, 265, 266, 267, 268, 269,
                           270, 271, 392, 393, 394, 395, 396, 397, 398, 399)
        elif self.compileParams.partition == 'wm_inf':
            chipAddrMap = (130, 129, 257, 258)
        elif self.compileParams.partition == 'nahuku08':
            chipAddrMap = (8, 9, 136, 137, 264, 265, 392, 393)

        self._axonFanout = np.zeros(self.numNeurons, dtype=int)
        self._synFanout = np.copy(self._axonFanout)

        for ng in self.neuronGroups.values():
            for ngs in ng.neuronGroupSlices:
                srcChipId = ngs.neuroCore.n2Core.parent.id
                # Initialize synMap
                synMap = ngs.neuroCore.n2Core.synapseMap
                for cg in ng.inputConnGroups.values():
                    inPopAxonId = ngs.getInputAxonId(cg)
                    axonToSynIdMap = ngs.getAxonToSynIdMap(cg)
                    synMap[inPopAxonId].population32MapEntry.cxBase = \
                        ngs.compartmentIds[0]
                    synMap[inPopAxonId].synapsePtr = axonToSynIdMap[0][0]
                    synMap[inPopAxonId].synapseLen = len(axonToSynIdMap[0])
                    synMap[inPopAxonId].popSize = len(axonToSynIdMap)

                # Initialize axonCfg
                axonCfg = ngs.neuroCore.n2Core.axonCfg
                axonCfgPtr = 2**12
                axonCfgLen = 0
                for cg in ng.outputConnGroups.values():
                    for toNg in cg.getToNeuronGroups(ng):
                        outAxIds = ngs.getOutputAxonIds(cg, toNg)
                        # Update axonCfgPtr and axonCfgLen for axonMap
                        axonCfgPtr = min(axonCfgPtr, min(outAxIds))
                        axonCfgLen += len(outAxIds)
                        # Create one axonCfg entry for each destination
                        # neuronGroupSlice
                        i = 0
                        for toNgSlice in toNg.neuronGroupSlices:
                            dstChipId = toNgSlice.neuroCore.n2Core.parent.id
                            dstCoreId = toNgSlice.neuroCore.n2Core.id
                            dstAxonId = toNgSlice.getInputAxonId(cg)
                            axCfgId = outAxIds[i]
                            if useMultiChip and srcChipId != dstChipId:
                                axonCfg[axCfgId].remote.configure(
                                    remoteChipId=chipAddrMap[dstChipId])
                                axCfgId += 1
                            axonCfg[axCfgId].pop32_0.coreId = dstCoreId
                            axonCfg[axCfgId].pop32_0.time = 0
                            axonCfg[axCfgId].pop32_0.axonId = dstAxonId
                            axCfgId += 1
                            axonCfg[axCfgId].pop32_1.configure()
                            i += 1

                # Initialize axonMap
                axonMap = ngs.neuroCore.n2Core.axonMap
                if self.modelParams.enableSpikes:
                    axonCfgLen *= self.compileParams.axonCfgSpacing
                    for cg in ng.outputConnGroups.values():
                        for toNg in cg.getToNeuronGroups(ng):
                            for toNgSlice in toNg.neuronGroupSlices:
                                dstChipId = toNgSlice.neuroCore.n2Core.parent.id
                                offBoard = (dstChipId in [((x << 7) | (y << 3) | z) for x in range(4)
                                                          for y in range(1, 2) for z in range(8)] and
                                            srcChipId in [((x << 7) | (y << 3) | z) for x in range(4)
                                                          for y in range(2, 3) for z in range(8)]) or \
                                           (srcChipId in [((x << 7) | (y << 3) | z) for x in range(4)
                                                          for y in range(1, 2) for z in range(8)] and
                                            dstChipId in [((x << 7) | (y << 3) | z) for x in range(4)
                                                          for y in range(2, 3) for z in range(8)])
                                if offBoard and self.modelParams.disableOffboardSpikes:
                                    axonCfgLen = 0
                                    break
                            if axonCfgLen == 0:
                                break
                        if axonCfgLen == 0:
                            break
                else:
                    axonCfgLen = 0
                relNeuronIds = ngs.getRelativeNeuronIds()
                i = 0
                for cxId in ngs.compartmentIds:
                    axonMap[cxId].ptr = axonCfgPtr
                    axonMap[cxId].len = axonCfgLen
                    axonMap[cxId].atom = relNeuronIds[i]
                    i += 1

                # Initialize somaState of each compartment
                somaState = ngs.neuroCore.n2Core.somaState
                for cxId in ngs.compartmentIds:
                    somaState[cxId].configure(vth=self.modelParams.vth)

                # Initialize cxMetaState
                cxMetaState = ngs.neuroCore.n2Core.cxMetaState
                for cxId in ngs.compartmentIds:
                    groupId = math.floor(cxId / 4)
                    # Just set phase for all banks blindly to IDLE
                    cxMetaState[groupId].configure(phase0=2, somaOp0=3)
                    cxMetaState[groupId].configure(phase1=2, somaOp1=3)
                    cxMetaState[groupId].configure(phase2=2, somaOp2=3)
                    cxMetaState[groupId].configure(phase3=2, somaOp3=3)

                # Store axon and synapse fanout
                nIds = ngs.neuronIds
                self._axonFanout[nIds] = axonCfgLen // self.compileParams.axonCfgSpacing
                numNeuronsPerGroup = ngs.neuronGroup.numNeurons
                grpFanout = ngs.neuronGroup.numOutputConnections
                self._synFanout[nIds] = numNeuronsPerGroup-1 \
                    + numNeuronsPerGroup*(grpFanout-1)

    def _buildNetwork(self):
        """Builds the LCA network by configuring all neuroCore registers"""

        # Create N2Board with N2Chips and N2Cores
        # numChips, numCores, numSynChips = self._calcChipsCoresSynapses()
        numChips = len(list(self.chipToCoreMap.keys()))
        numCores = [len(coreList) for coreList in self.chipToCoreMap.values()]
        numSynChips = self._computeNumSynPerCorePerChip(numChips, numCores)
        self.board = N2Board(0, numChips, numCores, numSynChips)

        # Associate N2Cores to NeuroCore group
        for chipId, coreIdList in self.chipToCoreMap.items():
            i = 0
            for coreId in coreIdList:
                n2Core = self.board.n2Chips[chipId].n2Cores[i]
                self.neuroCores[coreId].n2Core = n2Core
                self.neuroCores[coreId].id = n2Core.id
                i += 1
        #
        # i = 0
        # for chipId in range(0, numChips):
        #     for coreId in range(0, numCores[chipId]):
        #         n2Core = self.board.n2Chips[chipId].n2Cores[coreId]
        #         self.neuroCores[i].n2Core = n2Core
        #         self.neuroCores[i].id = n2Core.id
        #         i += 1

        # Initialize registers
        self._buildSharedRegisters()
        self._buildDiscreteRegisters()

    def _scaleWeightsToWgtRange(self, wgtMat, minWgt, maxWgt):
        """Scales weight matrix to the weight range implied by the weight
        precision. The scaling is performed in such a way as to preserve
        the zero-centering of the weight matrix otherwise anti-correlated
        dictionary elements would turn into correlated dictionary elements
        and vice versa. Thus we scale the weights such that either positive
        or negative weights span the full dynamic range of the weight range.

        :param wgtMat: (numpy.array) Weight Matrix to be scaled
        :param minWgt: (int) minimum weight, decides the lower bound of scaling
        :param maxWgt: (int) maximum weight, decides the upper bound of scaling
        :returns wgtMat: (numpy.array) Scaled weight matrix, as an int array
        """

        wgtMat = wgtMat.round(10)
        mp = self.modelParams

        # Determine wgtPrecision
        if mp.useMixedWeights:
            # When using mixed weights, we loose 1 value bit for explicit sign
            # encoding
            wgtPrecision = mp.wgtPrecision - 1
        else:
            wgtPrecision = mp.wgtPrecision

        # Determine old and new wgtRange of wgtMat while preserving
        # zero-centering
        # minWgt = np.abs(np.min(wgtMat))
        # maxWgt = np.max(wgtMat)
        minWgt = np.abs(minWgt)
        maxWgt = np.abs(maxWgt)
        newWgtRange = 2 ** wgtPrecision - 1

        if maxWgt >= minWgt:
            # wgtRange is limited by maxWgt
            oldWgtRange = maxWgt
        else:
            # wgtRange is limited by minWgt
            oldWgtRange = minWgt

        # Scale weights to newWgtRange and preserve zero-centering
        # The final shift by wgtExp is needed for 8-bit MSB alignment
        wgtMat = (newWgtRange * wgtMat / oldWgtRange) * 2**(8-mp.wgtPrecision)

        return wgtMat.astype(int)

    def _separateDictionary(self):
        """Separates (n x m) dictionary into positive only (2n x 2m)
        dictionary:
            D -> D+  D-
                 D-  D+
        Where D+ and D- are given by:
            D+ = D(D>=0)
            D- = -D(D<0)
        """
        n, m = self._dictionary.shape

        dp = np.zeros((n, m), dtype=float)
        dm = np.zeros((n, m), dtype=float)
        idxp = self._dictionary >= 0
        idxm = self._dictionary < 0
        dp[idxp] = self._dictionary[idxp]
        dm[idxm] = -self._dictionary[idxm]

        d = np.zeros((2*n, 2*m), dtype=float)
        d[0:n, 0:m] = dp
        d[n:2*n, m:2*m] = dp
        d[0:n, m:2*m] = dm
        d[n:2*n, 0:m] = dm

        return d

    def _generateWeights(self):
        """Generate weight matrices for each connectivityGroup"""

        globalMin = 100000000
        globalMax = -100000000

        D = self._separateDictionary()
        Dtrans = D.transpose()
        diagIdx = np.where(np.eye(D.shape[0], dtype=bool))

        for cg in self.connectivityGroups.values():
            # Get a representative (fromNg, toNg) pair of connectivityGroup
            fromNg = list(cg.forwardConnections.keys())[0]
            toNg = cg.getToNeuronGroups(fromNg)[0]
            # Compute weight matrix of get transpose of inverse
            # connctivityGroup from toNg -> fromNg
            invCg = self._connGroupMap[(toNg, fromNg)]
            if invCg in self._cgToWeightsMap:
                wgtMat = self._cgToWeightsMap[invCg].transpose()
            else:
                fromPatch = self.patches[fromNg.id]
                toPatch = self.patches[toNg.id]
                # Compute dictionaries expanded to size of overlap space
                fromIdx, toIdx, spaceDim = fromPatch.getLinearOverlapIndices(
                    toPatch)
                fromD = np.zeros(
                    (spaceDim*2, fromNg.numNeurons), dtype=D.dtype)
                fromD[np.concatenate((fromIdx, fromIdx)), :] = Dtrans
                toD = np.zeros((toNg.numNeurons, spaceDim*2), dtype=D.dtype)
                toD[:, np.concatenate((toIdx, toIdx))] = D
                # Compute weight matrix
                wgtMat = np.matmul(toD, -fromD)
                if fromNg.id == toNg.id:
                    wgtMat[diagIdx] = 0
                # Update globalMin/Max for final normalization
                globalMin = round(min(globalMin, np.min(wgtMat)), 10)
                globalMax = round(max(globalMax, np.max(wgtMat)), 10)
            # Store weight matrix
            self._cgToWeightsMap[cg] = wgtMat

        # Scale all weight matrices globally to full dynamic weight range
        for cg, wgtMat in self._cgToWeightsMap.items():
            # ToDo: For performance: Get already scaled weight matrix from
            # inverse connectivityGroup or avoid storing the unscaled matrix
            # above altogether.
            wgtMat = self._scaleWeightsToWgtRange(wgtMat, globalMin, globalMax)
            self._cgToWeightsMap[cg] = wgtMat

    def _writeWeights(self):
        """Write weights to N2Core synapseMem registers."""

        # Loop over all NeuroCores
        for nc in self.neuroCores.values():
            core = nc.n2Core
            for cg in nc.connectionGroups:
                wgtMat = self._cgToWeightsMap[cg]
                # Get map from axId -> synIds for connectivityGroup on this core
                axToSynIds = nc.getAxonToSynIdMap(cg)
                # Get list of neuron ids that connectivityGroup connects to
                relNeuronIds = nc.getRelativeNeuronIds(cg)
                for axId, synIds in axToSynIds.items():
                    for synCtr, synId in enumerate(synIds):
                        relNeuronId = relNeuronIds[synCtr]
                        core.synapses[synId].configure(synFmtId=2,
                                                       CIdx=synCtr,
                                                       Wgt=wgtMat[relNeuronId, axId])
                        # core.synapses[synId].synFmtId = 2
                        # core.synapses[synId].CIdx = synCtr
                        # core.synapses[synId].Wgt = wgtMat[relNeuronId, axId]

    def _scaleBiasToRange(self, bias):
        """Scales bias-mantissa to user-specified precision, if provided
        (or to signed 13-bit range by default), with a 3-bit bias-exponent
        
        :param bias: (int) should be a scalar
        :returns biasMant: (int) scalar 
        :returns biasExp: (int) scalar
        """

        mp = self.modelParams

        biasExp = np.ceil(np.log2(np.abs(bias+1))) - (mp.biasPrecision - 1)
        biasExp = (biasExp >= 0)*biasExp
        biasMant = np.floor(bias / 2**biasExp)

        return np.int_(biasMant), np.int_(biasExp)

    def _separatePatchImg(self, img):
        """"""

        n, m = img.shape
        # ToDo: Why actually transpose image?
        img = np.reshape(img.T, (n*m, 1))

        sImgP = np.copy(img)
        sImgM = -np.copy(img)
        sImgP[img < 0] = 0
        sImgM[img >= 0] = 0

        return np.concatenate((sImgP, sImgM), axis=0)

    def _generateBiases(self):
        """Generates bias currents for each neuronGroup from input and
        dictionary.
        """

        # Initialize
        mp = self.modelParams
        numPatchPixels = mp.patchSizeX * mp.patchSizeY
        # dictionary = self.dictionary
        d = self._separateDictionary()

        # Extract patch image and compute bias current for neuronGroup
        for pId, patch in self.patches.items():
            mask = patch.getMask(mp.imgSizeX, mp.imgSizeY)
            patchImg = np.copy(self.input[mask])
            patchImg = np.reshape(patchImg, (mp.patchSizeY, mp.patchSizeX))
            # patchImg = np.reshape(patchImg.T, (numPatchPixels, 1))
            patchImg = self._separatePatchImg(patchImg)
            # bias0 = dictionary.dot(patchImg)
            bias0 = d.dot(patchImg)
            bias = np.round(
                ((bias0 - mp.regularizationFactor) * mp.biasScaling),).astype(
                int)
            self._biases[pId] = np.copy(bias)

    def _writeBiases(self):
        """Writes bias currents from each neuronGroup to N2Core registers"""

        for ng in self.neuronGroups.values():
            bias = np.copy(self._biases[ng.id])
            for ngs in ng.neuronGroupSlices:
                core = ngs.neuroCore.n2Core
                cxCtr = 0
                for relNId in ngs.getRelativeNeuronIds():
                    cId = ngs.compartmentIds[cxCtr]
                    if np.abs(bias[relNId]) < 2**(self.modelParams.biasPrecision-1):
                        core.cxCfg[cId].configure(bias=bias[relNId][0],
                                                  biasExp=0)
                    else:
                        biasMant, biasExpt = self._scaleBiasToRange(bias[relNId][0])
                        core.cxCfg[cId].configure(bias=biasMant,
                                                  biasExp=biasExpt)
                    cxCtr += 1

    def resetDynamicStates(self, stateNames):
        """Resets dynamic neuron states specified by stateNames.

        :param list<str> stateNames: List of strings specifying state names
        to reset. Allowed values are 'compartmentCurrent',
        'compartmentVoltage' and 'activity'.
        """

        stateMap = {
            'dendriteAccum': ('dendriteAccum', 'word'),
            'compartmentCurrent': ('cxState', 'u'),
            'compartmentVoltage': ('cxState', 'v'),
            'activity': ('somaState', 'a')
        }
        for stateName in stateNames:
            assert isinstance(stateName, str), "<stateName> must be a string."
            assert stateName in stateMap.keys(), "<stateName> must be " \
                                                 "'dendriteAccum', " \
                                                 "'compartmentCurrent', " \
                                                 "'compartmentVoltage' or " \
                                                 "'activity."

        # ToDo: Wasteful: Should be replaced by fastInit
        resetDendAccum = 'dendriteAccum' in stateNames
        resetActivity = 'activity' in stateNames
        resetCompartmentCurrent = 'compartmentCurrent' in stateNames
        resetCompartmentVoltage = 'compartmentVoltage' in stateNames
        for ng in self.neuronGroups.values():
            nId = 0
            for ngs in ng.neuronGroupSlices:
                n2c = ngs.neuroCore.n2Core
                for cId in ngs.compartmentIds:
                    # if resetDendAccum:
                    #    n2c.dendriteAccum[cId:2**13:1024].word = 0
                    if resetActivity:
                        n2c.somaState[cId].a = 0
                    if resetCompartmentCurrent:
                        n2c.cxState[cId].u = 0
                    if resetCompartmentVoltage:
                        n2c.cxState[cId].v = 0
                    nId += 1
        if resetDendAccum:
            for nc in self.neuroCores.values():
                da = nc.n2Core.dendriteAccum
                for i in range(0, 2**13):
                    da[i].word = 0
        self.board.push()

        return 0

    def _reconstructPatch(self, ngId, rates):
        """Reconstruct a single patch.

        :param ngId: (int) NeuronGroupId for neuron group corresponding to the patch
        :param rates: (numpy.array) 2D array of spiking rates of all neurons in the
                      neuron group at each time-step
        :param ntime: (int) number of time-steps for which the code was run

        :returns yyIds: (numpy.array) 2D array, row numbers of this reconstructed
                        patch in the full image reconstruction
        :returns xxIds: (numpy.array) 2D array, column numbers of this reconstructed
                        patch in the full image reconstruction
        :returns imgEvolution: (numpy.array) 2D array, time evolution of the reconstructed
                               patch image"""

        ntime = rates.shape[0]

        imgEvolution = np.dot(rates, self._separateDictionary())
        _, m = imgEvolution.shape
        assert m % 2 == 0
        m = int(m/2)
        imgEvolution = imgEvolution[:, 0:m] - imgEvolution[:, m:2*m]

        patch = self.patches[ngId]
        yIds = range(patch.y0, patch.y0 + patch.dy)
        xIds = range(patch.x0, patch.x0 + patch.dx)
        imgEvolution = imgEvolution.reshape((ntime, patch.dx, patch.dy))
        yyIds, xxIds = np.meshgrid(yIds, xIds)

        return yyIds, xxIds, imgEvolution

    # --------------------------------------------------------------------------
    # Interface
    def generateNetwork(self):
        """Generates the LCA network and allocates resources."""

        info('Generating network...')
        t0 = time.time()
        self._generatePatches()
        self._generateGroups()
        self._computeResourceRequirements()
        self._partitionNetwork()
        self._associateNetwork()
        self._buildNetwork()
        self._netIsGenerated = True
        info(' Done ({:.2f}s)'.format(time.time() - t0))

    def getWeights(self):
        """Return weight matrices of all connectivityGroups as as (n x n x m) \
        tensor where <n> is the numNeuronsPerGroup and <m> is the number of
        connectivityGroups."""

        assert len(self._cgToWeightsMap) > 0, \
            'Weights have not been generated yet.'
        n = self.resourceRequirements.numNeuronsPerGroup
        weights = np.zeros((n, n, self.numConnectivityGroups), dtype=int)
        for i, cg in enumerate(self.connectivityGroups.values()):
            weights[:, :, i] = np.copy(self._cgToWeightsMap[cg])

        return weights

    def setWeights(self, weights):
        """Sets weights of all connectivityGroups from (n x n x m) weights \
        tensor where <n> is the numNeuronsPerGroup and <m> is the number of
        connectivityGroups."""

        for i, cg in enumerate(self.connectivityGroups.values()):
            self._cgToWeightsMap[cg] = weights[:, :, i]

    def setDictionary(self, dictionary, weights=None):
        """Sets a new dictionary, recomputes weight matrices for each
        connectivityGroup and updates neuroCores.

        :param numpy.ndarray dictionary: Dictionary matrix of size (dictSize \
        x patchSizeX*patchSizeY).
        :param np.ndarray weights: (n x m x numConnGrps) array with weights \
        for all connectivityGroups. If None, weights will be computed from \
        dictionary matrix for all connectivityGroups. Otherwise precomputed \
        weights will be used.
        """
        # Validate inputs
        mp = self.modelParams
        assert self._netIsGenerated, \
            'Network must be generated before dictionary can be set.'
        dictShape = (mp.dictSize,
                     mp.patchSizeX * mp.patchSizeY)
        assert dictionary.shape == dictShape, \
            '<dictionary> has invalid shape. Must be of size (%d, %d) ' \
            'but has size (%d, %d).' \
            % (dictShape[0], dictShape[1],
               dictionary.shape[0], dictionary.shape[1])

        info('Setting up dictionary and ')
        t0 = time.time()

        # Generate weight matrices and write weights to N2Cores
        self.dictionary = np.copy(dictionary)
        if weights is None:
            info('\tgenerating weights...')
            # if not self.compileParams.disableNeuronUpdates:
            self._generateWeights()
        else:
            info('\tsetting weights...')
            # if not self.compileParams.disableNeuronUpdates:
            self.setWeights(weights)
        # if not self.compileParams.disableNeuronUpdates:
        self._writeWeights()
        info(' Done ({:.2f}s)'.format(time.time() - t0))
        self._dictIsSet = True

    def setInput(self, input):
        """Sets new input image, recomputes bias currents and updates N2Cores. \
        For each neuronGroup, the bias current is computed as follows:

        bias = (dict*patchImg - regularizationFactor) * biasScaling

        :param numpy.ndarray input: Input image of size (imgSizeY x imgSizeX).
        """
        # Validation
        assert self._netIsGenerated, \
            'Network must be generated before input can be set.'
        assert self._dictIsSet, \
            '<dictionary> must be set before input.'
        assert isinstance(input, np.ndarray), \
            '<input> must be an numpy array.'
        mp = self.modelParams
        assert input.shape == (mp.imgSizeY, mp.imgSizeX), \
            '<input> has invalid size. Must be of size (%d, %d) but has size ' \
            '(%d, %d).' \
            % (mp.imgSizeY, mp.imgSizeX, input.shape[0], input.shape[1])

        info('Setting up input image and generating bias currents...')
        t0 = time.time()

        # Generate bias currents and write to N2Cores
        self.input = np.copy(input)
        self._generateBiases()
        self._writeBiases()
        info(' Done ({:.2f}s)\n'.format(time.time() - t0))
        self._inputIsSet = True

    def createProbes(self, stateName, neuronGroups=None):
        """Creates probes for all neurons within list of neuronGroups
        for specified state.

        :param str stateName: The name of the state to probe. Supported \
        states are 'compartmentCurrent', 'compartmentVoltage' and 'activity.
        :param list neuronGroups: A list of neuronGroup objects. \
        :return OrderedDict: Mapping from neuronGroupId to list of probes for \
        all neurons within neuronGroup.
        """

        stateMap = {
            'compartmentCurrent': ('cxState', 'u'),
            'compartmentVoltage': ('cxState', 'v'),
            'activity': ('somaState', 'a')
        }
        assert isinstance(stateName, str), "<stateName> must be a string."
        assert stateName in stateMap.keys(), "<stateName> must be " \
                                             "'compartmentCurrent', " \
                                             "'compartmentVoltage' or " \
                                             "'activity."
        assert isinstance(neuronGroups, Iterable), \
            'neuronGroups must be a list of NeuronGroup objects.'

        mp = self.modelParams

        assert mp.numStepsPerEpoch >= mp.probeInterval
        assert mp.numStepsPerEpoch % mp.probeInterval == 0, \
            'numStepsPerEpoch must be a multiple of probeInterval.'

        dt = mp.probeInterval
        offset = mp.probeOffset
        regName, stateName = stateMap[stateName]

        mon = self.board.monitor
        pc = IntervalProbeCondition(dt, offset)
        probes = OrderedDict()
        for ng in neuronGroups:
            groupProbes = []
            for ngs in ng.neuronGroupSlices:
                cIds = ngs.compartmentIds
                register = ngs.neuroCore.n2Core.__getattribute__(regName)
                probe = mon.probe(register, cIds, stateName, pc)
                groupProbes += probe
            probes[ng.id] = groupProbes

        return probes

    def createProbesOnCxIds(self, stateName, neuronIds=None):
        """Creates probes for all compartments provided in cxIds
        for specified state.

        :param str stateName: The name of the state to probe. Supported \
        states are 'compartmentCurrent', 'compartmentVoltage' and 'activity.
        :param list cxIds: A list of compartment IDs . \
        :return list(): list of probes for all specified compartments.
        """

        stateMap = {
            'compartmentCurrent': ('cxState', 'u'),
            'compartmentVoltage': ('cxState', 'v'),
            'activity': ('somaState', 'a')
        }
        assert isinstance(stateName, str), "<stateName> must be a string."
        assert stateName in stateMap.keys(), "<stateName> must be " \
                                             "'compartmentCurrent', " \
                                             "'compartmentVoltage' or " \
                                             "'activity."
        assert isinstance(neuronIds, list), \
            'cxIds must be a list of compartment IDs to be probed.'

        mp = self.modelParams

        assert mp.numStepsPerEpoch >= mp.probeInterval
        assert mp.numStepsPerEpoch % mp.probeInterval == 0, \
            'numStepsPerEpoch must be a multiple of probeInterval.'

        dt = mp.probeInterval
        offset = mp.probeOffset
        regName, stateName = stateMap[stateName]

        mon = self.board.monitor
        pc = IntervalProbeCondition(dt, offset)
        probes = OrderedDict()
        probedCxIds = OrderedDict()
        for ng in list(self.neuronGroups.values()):
            groupProbes = []
            groupProbedCxIds = []
            for ngs in ng.neuronGroupSlices:
                neuronIdToCxIdMap = dict(zip(ngs.neuronIds, ngs.compartmentIds))
                neuronSetThisNgs = set(neuronIds) & set(ngs.neuronIds)
                if neuronSetThisNgs:
                    cxSetThisNgs = [neuronIdToCxIdMap[nId]
                                    for nId in neuronSetThisNgs]
                    register = ngs.neuroCore.n2Core.__getattribute__(regName)
                    sliceProbes = mon.probe(register, list(cxSetThisNgs), stateName, pc)
                    ngSpecificIds = list(np.array(list(neuronSetThisNgs)) % len(ng.neuronIds))
                    for j, p in enumerate(sliceProbes):
                        p.ngSpecificId = ngSpecificIds[j]
                    groupProbes += sliceProbes
                    groupProbedCxIds += list(cxSetThisNgs)
            if groupProbes:
                probes[ng.id] = groupProbes
                probedCxIds[ng.id] = groupProbedCxIds

        return probes, probedCxIds

    def run(self):
        """Runs LcaNet for numEpochs*numStepsPerEpoch time steps."""

        numEpochs = self.modelParams.numEpochs
        numStepsPerEpoch = self.modelParams.numStepsPerEpoch

        if self.verbosity > VerbosityLevel.LOW:
            info('Running LcaNet for %d epochs and %d stepsPerEpoch... ' %
                  (numEpochs, numStepsPerEpoch))

        # Start the lcanet.board.executor() if not already started:
        if not self.board.executor.hasStarted():
            self._startDriver()

        startTime = time.time()
        runStartTime = resetStartTime = 0
        for epoch in range(numEpochs):
            # Execute network
            if self.verbosity > VerbosityLevel.MID:
                info("  Running epoch %d... " % (epoch))
                runStartTime = time.time()
            self.board.run(numStepsPerEpoch)
            if self.verbosity > VerbosityLevel.MID:
                info("  Done (%.2fs)" % (time.time() - runStartTime))

            # Reset state
            if self.verbosity > VerbosityLevel.MID:
                info("  Resetting activity... ")
                resetStartTime = time.time()
            self.resetDynamicStates(['activity'])
            if self.verbosity > VerbosityLevel.MID:
                info("  Done (%.2fs)" % (time.time() - resetStartTime))
            self._numEpochs += 1

        if self.verbosity > VerbosityLevel.LOW:
            info("Done %.2fs" % (time.time() - startTime))

    def extractProbeData(self, probes, fromEpoch, toEpoch):
        """Extracts timeSeries data of all probes within time window defined
        by fromEpoch/toEpoch. Epochs refer to execution periods of duration
        numStepsPerEpoch after which activity traces are read out and reset.

        :param dict probes: Dictionary mapping neuron group ids to list of
        probes.
        :param int fromEpoch: Specifies epoch at which extraction should be
        begin. Must be >=1.
        :param int toEpoch: Specifies epoch until which to extract data.
        :return time: (numProbeSteps,) vector of time steps for each probe data
        point.
        :rtype: numpy.ndarray
        :return probeData: (numProbeSteps, numNeurons) numpy.ndarray or the
        probed data for all neuron groups.
        """

        assert 1 <= fromEpoch <= toEpoch, 'Illegal epoch selection. Must ' \
                                          'satisfy 1 <= fromEpoch <= toEpoch.'

        # Compute time window from which to extract probe data
        mp = self.modelParams
        tMin = (fromEpoch-1)*mp.numStepsPerEpoch + 1
        tMax = toEpoch*mp.numStepsPerEpoch

        # Extract probe data
        time = probeData = None
        mon = self.board.monitor
        for ngId, groupProbes in probes.items():
            ng = self.neuronGroups[ngId]
            time, tmpData = mon.probesToNumpy(groupProbes, tMin, tMax)
            if tmpData.shape[1] != len(ng.neuronIds):
                data = np.zeros((len(time), len(ng.neuronIds)))
                probedCxIds = []
                for probe in groupProbes:
                    probedCxIds += [probe.ngSpecificId]  # [int(probe.nodeId % len(ng.neuronIds))]
                data[:, probedCxIds] = tmpData
            else:
                data = tmpData
            if probeData is None:
                probeData = np.zeros((len(time), self.numNeurons))
            probeData[:, ng.neuronIds] = data

        return time, probeData

    def _computeNumSpikes(self, time, spikeCounts):
        """Computes cumulative number of spikes from spikeCounts within
        individual epochs.
        At the end of each epoch, activity traces are read out to avoid
        saturation and then activity traces are reset. This function adds
        spikeCounts from subsequent epochs up to produce a continuous
        cumulative spike count without resets."""

        mp = self.modelParams

        # Extract number of spikes at end of each epoch and compute total
        # number of spikes regardless of spikeCount reset at end of every epoch
        numSpikesAtEpoch = np.zeros(spikeCounts.shape, dtype=int)
        for i, t in enumerate(time):
            # Subtract time[0] from t to translate the time axis
            if i > 0 and (t - time[0]) % mp.numStepsPerEpoch == 0:
                numSpikesAtEpoch[i, :] = spikeCounts[i - 1, :]
        numSpikes = np.ceil(spikeCounts) + np.cumsum(numSpikesAtEpoch, 0)

        return numSpikes

    def _computeRates(self, time, numSpikes):
        """Computes the sparse rate code from probe time and spike counts.

        :param numpy.ndarray time: Probe times
        :param numpy.ndarray: numSpikes: Cumulative spike count.

        :return: (numProbeSteps, numNeurons) matrix of rates
        :rtype: numpy.ndarray
        """

        mp = self.modelParams
        time = np.reshape(time, (len(time), 1))
        time = time - time[0] + mp.probeInterval
        rates = numSpikes / time / mp.timePerStep

        return rates

    def _computeInstantaneousRates(self, time, numSpikes):
        pass

    def extractSparseCode(self, activityProbes, fromEpoch, toEpoch):
        """Extracts the sparse rate code from activity probes within the time
         window specified by fromEpoch/toEpoch."""

        time, data = self.extractProbeData(activityProbes, fromEpoch, toEpoch)
        numSpikes = self._computeNumSpikes(time, data)
        return time, numSpikes, self._computeRates(time, numSpikes)

    def reconstructInput(self, sparseCode):
        """Reconstructs input from cumulative rates.

        :param numpy.ndarray sparseCode: Cumulative (numSamples x numNeurons) \
        rate matrix.
        :return: (numSamples x imageSizeY x imageSizeX) temporal evolution of \
        reconstruction.
        :rtype: numpy.ndarray
        """
        nProbeSteps = sparseCode.shape[0]
        mp = self.modelParams
        reconstruction = np.zeros((nProbeSteps, mp.imgSizeY, mp.imgSizeX))
        for ngId, ng in self.neuronGroups.items():
            rates = sparseCode[:, ng.neuronIds]
            rows, cols, imgEvolution = self._reconstructPatch(ngId, rates)
            reconstruction[:, rows, cols] += imgEvolution

        return reconstruction

    def evaluateErrorMetrics(self, numSteps, reconstruction, rates):
        """Calculates reconstruction error, objective function of the
        L1-regularisation problem, and sparsity of LcaNet.

        :param int numSteps: Number of time-steps for which the network was run.
        :param numpy.ndarrray reconstruction: 3D array, containing 2D output \
        at each time-step.
        :param numpy.ndarray rates: 2D array, containing spiking \
        cumulative rates of all neurons at each time-step .
        :returns numpy.ndarray error: 1D array, reconstruction error as at \
        every time-step.
        :returns numpy.ndarray objective: 1D array, objective value at every \
        time-step.
        :returns numpy.array sparsity: 1D array, sparsity of the network at \
        every time-step.
        """
        mp = self.modelParams

        original = np.tile(self.input, (numSteps, 1, 1))
        error = np.sum(np.sum((reconstruction - original) *
                              (reconstruction - original), axis=2), axis=1)
        error = error.reshape((1, numSteps))
        error = error.flatten()

        objective = error/2 + \
            mp.regularizationFactor * np.sum(np.abs(rates), axis=1)
        objective = objective.flatten()

        sparsity = np.sum(np.abs(rates) > 0, axis=1) / rates.shape[1] * 100
        sparsity = sparsity.flatten()

        return error, objective, sparsity

    def solve(self):
        """Solves the sparse coding problem: argmin_z |x - D'*z|**2 + lambda*|z|

        :return: Solution object of sparse coding problem.
        :rtype: Solution
        """

        mp = self.modelParams

        if mp.probeActivity and self._activityProbes is None:
            if mp.probeCxIdList is not None:
                self._activityProbes, self._probedCxIds = self.createProbesOnCxIds('activity',
                                                                                   mp.probeCxIdList)
            else:  # Probe all Cx
                self._activityProbes = self.createProbes('activity',
                                                         list(self.neuronGroups.values()))

        self.run()
        if mp.probeActivity:
            toEpoch = self._numEpochs
            fromEpoch = self._numEpochs - self.modelParams.numEpochs + 1
            time, numSpikes, rates = self.extractSparseCode(self._activityProbes,
                                                            fromEpoch, toEpoch)
            reconstruction = self.reconstructInput(rates)
            numSamplePoints = reconstruction.shape[0]
            metrics = self.evaluateErrorMetrics(numSamplePoints,
                                                reconstruction,
                                                rates)
        else:
            time = np.ndarray([0], dtype=int)
            numSpikes = np.zeros((1, self.numNeurons), dtype=int)
            rates = np.zeros((1, self.numNeurons), dtype=int)
            reconstruction = np.zeros((1, mp.imgSizeX, mp.imgSizeY), dtype=int)
            metrics = (np.zeros((1, 1), dtype=int),
                       np.zeros((1, 1), dtype=int),
                       np.zeros((1, 1), dtype=int))

        return Solution(self, time, numSpikes, rates, reconstruction,
                        metrics[0], metrics[1], metrics[2])

    # --------------------------------------------------------------------------
    # Utilities
    def genFullWgtMat(self):
        """Generates full weight matrix of individual connectivityGroup
        weight matrices."""
        grandWeightMatrix = np.zeros(
            (self.numNeurons, self.numNeurons), dtype=int)

        # Only meddle with the grand weight matrix if net is generated.
        # Else, leave it be zero
        if self._netIsGenerated:
            # Loop over all ConnectivityGroups
            for cgid, cg in self.connectivityGroups.items():
                localWeightMatrix = self._cgToWeightsMap[cg]
                # Loop over all forward connections
                for ng1, ng2List in cg.forwardConnections.items():
                    # Loop over all "to" neurons
                    for ng2 in ng2List:
                        assert ng1.numNeurons == ng2.numNeurons, \
                            'Number of atoms must be the same across ' \
                            'dictionaries corresponding to different ' \
                            'NeuronGroups.'
                        # Assign the computed local weight matrix to a block in
                        # the grandWeightMatrix of the LcaNet
                        idxMat = np.meshgrid(ng2.neuronIds, ng1.neuronIds)
                        grandWeightMatrix[idxMat] = localWeightMatrix

        return grandWeightMatrix

    def findConnectivityGroup(self, fromNg, toNg):
        """Finds connectivityGroup that connects neuronGroup fromNg to
        neuronGroup toNg.

        :param NeuronGroup fromNg: Source neuronGroup.
        :param NeuronGroup toNg: Destination neuronGroup.
        :return: ConnectionGroup connecting the two neuronGroups or None if \
        neuronGroups are not connected.
        :rtype: ConnectivityGroup
        """
        fCg = None
        for cg in self.connectivityGroups.values():
            if fromNg in cg.forwardConnections.keys():
                if toNg in cg.forwardConnections[fromNg]:
                    fCg = cg
        return fCg

    # ToDo: Use findConnectivityGroup
    def getWgtMat(self, fromNgId, toNgId):
        """Returns the weight matrix connecting the two neuronGroups with id
        fromNgId to toNgId.

        :param int fromNgId: Source neuronGroupId.
        :param int toNgId: Destination neuronGroupId
        :return: Weight matrix.
        :rtype: numpy.ndarray
        """
        if self._netIsGenerated:
            for cg in self.connectivityGroups.values():
                for ng1, ng2list in cg.forwardConnections.items():
                    for ng2 in ng2list:
                        if ng1.id == fromNgId and ng2.id == toNgId:
                            return self._cgToWeightsMap[cg]

    # --------------------------------------------------------------------------
    # Visualization
    def showDictionary(self, figId=0):
        """Visualizes dictionary elements on a 2D grid as 2D images."""

        numRows = int(math.sqrt(self.modelParams.dictSize))
        numCols = int(math.ceil(self.modelParams.dictSize / numRows))

        dy = self.modelParams.patchSizeY
        dx = self.modelParams.patchSizeX
        bigImgDy = numRows * (dy + 1) - 1
        bigImgDx = numCols * (dx + 1) - 1
        background = np.max(self.dictionary)
        bigImg = np.ones((bigImgDy, bigImgDx)) * background

        i = 0
        for r in range(numRows):
            yId = range(r * (dy + 1), (r + 1) * (dy + 1) - 1)
            for c in range(numCols):
                dictElem = np.reshape(self.dictionary[i, :], (dy, dx))
                xId = range(c * (dx + 1), (c + 1) * (dx + 1) - 1)
                idMat = np.meshgrid(yId, xId)
                bigImg[tuple(idMat)] = dictElem
                i += 1
                if i == self.modelParams.dictSize:
                    break
            if i == self.modelParams.dictSize:
                break

        plt.figure(figId)
        plt.imshow(bigImg, cmap='gray')
        plt.colorbar()
        plt.title('Dictionary elements')
        plt.show()

    def showInput(self, figId=5):
        """Visualizes the input image."""

        plt.figure(figId)
        plt.imshow(self.input, cmap='gray')
        plt.title('Input')
        plt.colorbar()
        plt.show()

    # ToDo: Refactor in terms of other methods.
    def showNGtoNGWgtMat(self, fromNgId, toNgId, axes=None, saveToFile=False):
        """Plot the weight matrix from fromNgId to toNgId"""

        if self._netIsGenerated:
            for cg in self.connectivityGroups.values():
                for ng1, ng2list in cg.forwardConnections.items():
                    for ng2 in ng2list:
                        if ng1.id == fromNgId and ng2.id == toNgId:
                            if saveToFile:
                                filename = 'localWgtMat_' + str(
                                    fromNgId) + 'to' + str(toNgId) + '.dat'
                                np.savetxt(filename,
                                           self._cgToWeightsMap[cg],
                                           fmt='%d')
                            elif axes is not None:
                                plt.gca()
                                img = axes.imshow(self._cgToWeightsMap[cg])
                                plt.show()
                                return img

    def showPatchCoverage(self):
        """Plots the patch coverage."""
        cov = self._computePatchCoverage()
        plt.imshow(cov)
        plt.colorbar()
        plt.show()

    def showOverlapMasks(self):
        """Shows the overlapMasks of all connectivityGroups. An overlapMask
        is a binary mask of the OR of two neuronGroup's receptive fields
        with respect to the input space. If the receptive fields do not
        overlap, there exists not connectivityGroup and no overlapMask."""

        # Find shape of largest mask in order to plot all masks in
        # same-sized frame
        largestShape = [0, 0]
        for cgId in self._overlapMasks:
            currentShape = self._overlapMasks[cgId].shape
            if currentShape[0] > largestShape[0]:
                largestShape[0] = currentShape[0]
            if currentShape[1] > largestShape[1]:
                largestShape[1] = currentShape[1]
        i = 1
        # Plot overlapMasks
        plt.figure(0)
        frameMask = np.zeros(largestShape, bool)
        numMasks = len(self._overlapMasks)
        numCols = math.ceil(math.sqrt(numMasks))
        numRows = math.ceil(numMasks/numCols)
        for cgId, mask in self._overlapMasks.items():
            ax = plt.subplot(numRows, numCols, i)
            ax.xaxis.set_visible(False)
            ax.yaxis.set_visible(False)
            frameMask[0:mask.shape[0], 0:mask.shape[1]] = mask
            plt.imshow(frameMask)
            frameMask[:] = False
            i += 1
        plt.show()

    def showGroupConnectivity(self):
        """Shows a (numNeuronGroups x numNeuronGroups) matrix where each
        element specifies the id of the connectivityGroup connecting the
        neuronGroups."""

        # Initialize groupConns mask with slightly non-zero background such
        # that cg[0] can be distinguished from others
        grpConns = np.zeros((self.numNeuronGroups, self.numNeuronGroups)) \
            - 1 - int(self.numConnectivityGroups/4)
        for i, fromNg in self.neuronGroups.items():
            for j, cg in fromNg.outputConnGroups.items():
                assert len(cg.forwardConnections[fromNg]) == 1, \
                    'fromNg[%d] connects more than once via cg[%d]' %\
                    (fromNg.id, cg.id)
                for toNg in cg.forwardConnections[fromNg]:
                    # print('fromNg[%d] -> cg[%d] -> toNg[%d]' %
                    #      (fromNg.id, cg.id, toNg.id))
                    if grpConns[toNg.id, fromNg.id] < 0:
                        grpConns[toNg.id, fromNg.id] = 0
                    grpConns[toNg.id, fromNg.id] += cg.id
        plt.figure(1)
        plt.imshow(grpConns)
        plt.xlabel('fromNeuronGroupId')
        plt.ylabel('toNeuronGroupId')
        plt.title('NeuronGroupConnectivity')
        plt.colorbar()
        plt.show()

    def showFullWgtMat(self):
        """Plot the full weight matrix of all connectivityGroups."""

        grandWeightMatrix = self.genFullWgtMat()
        np.savetxt('fullwgtmat.dat', grandWeightMatrix, fmt='%d')
        # Create an image plot
        plt.figure(3)
        plt.imshow(grandWeightMatrix)
        plt.xlabel('From Neuron')
        plt.ylabel('To Neuron')
        plt.title('Grand Weight Matrix')
        plt.colorbar()
        plt.show()

        return grandWeightMatrix

    def pickleDumpLcaNet(self, filepath):
        import pickle
        with open(filepath, 'wb') as f:
            pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
