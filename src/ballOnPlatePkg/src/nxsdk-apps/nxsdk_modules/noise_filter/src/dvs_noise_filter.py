# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

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

import os
import nxsdk.api.n2a as nx
import numpy as np
import scipy.sparse as sps
import inspect


class DVSNoiseFilter(nx.NetModule):
    """DVS noise filtering module. Removes isolated noise events from a DVS datastream
    """

    def __init__(self,
                 net=None,
                 pix2InputCGMapping=None,
                 dimX=240,
                 dimY=180,
                 dimP=2,
                 timePerTimestep=1e-3,
                 enableDurationTimesteps=6,
                 refractoryDurationTimesteps=6):
        """Initializer arguments specify the noise filter dimensions and parameters.
        :param NxNet net: Optional, specifies the network to create the noise filter in
        :param NDarray(int) pix2InputCGMapping: Optional, specifies how pixel [x,y,p] addresses correspond to linear indices. 
                            To get the linear address for [x,y,p], use pix2InputCGMapping[x,y,p]
        :param int dimX: X dimension of DVS sensor
        :param int dimY: Y dimension of DVS sensor
        :param int dimP: Polarity dimension of DVS sensor
        :param float timePerTimestep: Desired wall time in seconds for a single Loihi Timestep
        :param int enableDurationTimesteps: How long (in Loihi timesteps) does a spike enable neighbouring compartments for 
        :param int refractoryDurationTimesteps: How long (in Loihi timesteps) should neuron refractory period be 
        """

        super().__init__(net)

        # Calculate how many microseconds per Loihi timestep
        self._microsecondsPerTimestep = int(timePerTimestep*1e6)

        self._xResolution = dimX
        self._yResolution = dimY
        self._pResolution = dimP

        self._numPixels = dimX*dimY*dimP

        isg = self.net.createInputStubGroup(
            size=self.numPixels, name='DVS raw')
        self.addInput(isg, 'rawDVS')

        # if no pixel mapping was specified, use the default pixel mapping
        if pix2InputCGMapping is not None:
            (x, y, z) = pix2InputCGMapping.shape
            if x != dimX or y != dimY or p != dimP:
                raise ValueError(
                    'Pixel mapping must be of size (dimX, dimY, dimP)')
            else:
                self._pixInputMapping = pix2InputCGMapping
        else:
            (x, y, p) = np.meshgrid(np.arange(dimX),
                                    np.arange(dimY), np.arange(dimP))
            self._pixInputMapping = self.pix2linear(x, y, p)

        # Strictness of the filter. Lower is stricter (less signal, less noise)
        self._enableDurationTimesteps = enableDurationTimesteps

        self._refractoryDurationTimesteps = refractoryDurationTimesteps

        self._cg = self._setupCompartments()

        self.addOutput(self._cg, 'filteredDVS')

        self._setupConnectionPrototypes()

        self._connectNetwork(self.inputs.rawDVS)

    @property
    def xResolution(self):
        """Horizontal resolution in pixels"""
        return self._xResolution

    @property
    def yResolution(self):
        """Vertical resolution in pixels"""
        return self._yResolution

    @property
    def pResolution(self):
        """Number of different polarities"""
        return self._pResolution

    @property
    def numPixels(self):
        """Number of pixels"""
        return self.xResolution*self.yResolution*self.pResolution

    @property
    def enableDurationTimesteps(self):
        return self._enableDurationTimesteps

    @property
    def refractoryDurationTimesteps(self):
        return self._refractoryDurationTimesteps

    def pix2linear(self, x, y, p):
        """A helper function to map pixels to linear indices
        """
        return x*self._yResolution*self._pResolution + y*self._pResolution + p

    def linear2pix(self, lin):
        """A helper function to map linear indices to pixels
        """
        p = lin % self._pResolution
        lin = (lin - p)/self._pResolution

        y = lin % self._yResolution
        lin = (lin - y)/self._yResolution

        x = lin % self._xResolution

        return x, y, p

    def _validPix(self, x, y, p):
        """A helper function to find indices of valid pixels (i.e. pixels addresses that are within range)
        """
        inValidIndices = \
            + np.less(x, 0) \
            + np.less(y, 0) \
            + np.less(p, 0) \
            + np.greater_equal(x, self._xResolution) \
            + np.greater_equal(y, self._yResolution) \
            + np.greater_equal(p, self._pResolution)

        return np.equal(inValidIndices, 0)

    def _setupCompartments(self):
        """Sets up a compartments group with one compartment per input compartment group size
        """

        # Compartment group parameters
        vThMant = 255
        compartmentCurrentDecay = 0
        compartmentVoltageDecay = 4096

        cxProto = nx.CompartmentPrototype(vThMant=vThMant,
                                          compartmentCurrentDecay=compartmentCurrentDecay,
                                          compartmentVoltageDecay=compartmentVoltageDecay,
                                          refractoryDelay=self._refractoryDurationTimesteps)

        # pixel (x,y,p) will map to cx[pix2linear(x,y,p)]
        cx = self._net.createCompartmentGroup(size=self._numPixels,
                                              prototype=cxProto
                                              )

        return cx

    def _setupConnectionPrototypes(self):
        """Sets up 3 types of connection prototypes:
        1: for 1-1 mapping from input compartments to internal compartments
        2: for 1-many mapping from input compartments to internal compartments (enable signal)
        3: for lateral inhibitory connections within internal compartment group
        """
        # Connection from pixel to corresponding compartment
        self._connProtoInput = nx.ConnectionPrototype(weight=255,
                                                      signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY,
                                                      delay=1,
                                                      postSynResponseMode=nx.SYNAPSE_POST_SYN_RESPONSE_MODE.BOX
                                                      )

        # Connection from pixel to neighbouring compartments, enabling them
        self._connProtoEnable = nx.ConnectionPrototype(weight=1,
                                                       signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY,
                                                       delay=self._enableDurationTimesteps,
                                                       postSynResponseMode=nx.SYNAPSE_POST_SYN_RESPONSE_MODE.BOX
                                                       )

        # Inhibitory connection between pixels at the same location, but of different polarities
        # to realize mutual refractory period
        self._connProtoMutualRefrac = nx.ConnectionPrototype(weight=-255,
                                                             signMode=nx.SYNAPSE_SIGN_MODE.INHIBITORY,
                                                             delay=self._refractoryDurationTimesteps,
                                                             postSynResponseMode=nx.SYNAPSE_POST_SYN_RESPONSE_MODE.BOX
                                                             )

    def _connectNetwork(self, inputConnector):
        """Wires up connections between the input compartmentGroup (inputConnector) and internal compartments (self._cg), 
        as well as lateral inhibition connections within self.Cout

        :param compartmentGroup inputConnector: The source of the DVS spikes to be filtered.
        """

        # Wire up the direct input connection (one-to-one connections between source compartments and internal compartments)
        (xx, yy, pp) = np.meshgrid(np.arange(self._xResolution),
                                   np.arange(self._yResolution), 
                                   np.arange(self._pResolution))
        destPix = self.pix2linear(xx, yy, pp)

        connectionMask = sps.identity(self._numPixels)

        inputConnector.connect(self._cg,
                               prototype=self._connProtoInput,
                               connectionMask=connectionMask
                               )

        # Wire up the enable signals (one-to-many connections from source compartments to a group of internal compartments)
        (xx, yy) = np.meshgrid(np.arange(self._xResolution), 
                               np.arange(self._yResolution))

        connectionMask = sps.coo_matrix((self._numPixels, self._numPixels))
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for pSrc in range(self._pResolution):
                    for pDst in range(self._pResolution):
                        if dx!=0 or dy!=0 or pSrc!=pDst:
                            x = xx + dx
                            y = yy + dy

                            # Make sure the target compartments are valid (within range)
                            validIndices = self._validPix(x, y, pDst)

                            destPix = self.pix2linear(x, y, pDst)
                            destPix = destPix[validIndices]

                            sourcePix = self.pix2linear(xx, yy, pSrc)
                            sourcePix = sourcePix[validIndices]

                            connectionMask = connectionMask + sps.coo_matrix((np.ones(sourcePix.shape).flat, (destPix.flat, sourcePix.flat)),
                                                                             shape=(self._numPixels, self._numPixels))


        inputConnector.connect(self._cg,
                               prototype=self._connProtoEnable,
                               connectionMask=connectionMask
                               )

        # Wire up the mutual inhibition within internal compartments
        # one-to-many mapping between polarities sharing an x-y location
        # in our case there are only 2 polarities, so it collapses
        # to a one-to-one mapping
        connectionMask = sps.coo_matrix((self._numPixels, self._numPixels))

        # First make a connection mask where each compartment wires to all compartments
        # with the same x-y location (including itself)
        for pSrc in range(self._pResolution):
            for pDst in [ii for ii in range(self._pResolution) if ii != pSrc]:
                sourcePix = self.pix2linear(xx, yy, pSrc)
                destPix = self.pix2linear(xx, yy, pDst)
                connectionMask = connectionMask + sps.coo_matrix((np.ones(sourcePix.shape).flat, (destPix.flat, sourcePix.flat)),
                                                                 shape=(self._numPixels, self._numPixels))
        

        self._cg.connect(self._cg,
                         prototype=self._connProtoMutualRefrac,
                         connectionMask=connectionMask
                         )