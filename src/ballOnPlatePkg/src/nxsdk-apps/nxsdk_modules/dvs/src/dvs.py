# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2019-2021 Intel Corporation.
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

"""Support for DVS sensors, both live and from file."""

from nxsdk.graph.nxinputgen.dvsinputgen import DVSSpikeGenerator 
from nxsdk.net.process.dvsspikegen import DVSSpikeGen
import os
import nxsdk.api.n2a as nx
import numpy as np
import scipy.sparse as sps
import inspect


class DVS(nx.NetModule):
    """DVS module. Provides support for DVS sensors.
    
    .. note:: This is in Alpha and likely to change.
    
    .. warning:: The live DVS module requires that DVS.setupSnips(board) be called. This will instantiate a Spiking phase snip on Chip 0 Lakemont 0, which prevents the user from using their own snip in the Spiking phase on the same Lakemont and Chip (just use another Lakemont).
    """

    def __init__(self,
                 net=None,
                 dimX=240,
                 dimY=180,
                 dimP=2,
                 isTestMode=False):
        """
        :param NxNet net: Optional, specifies the network to create the noise filter in
        :param int dimX: X dimension of DVS sensor
        :param int dimY: Y dimension of DVS sensor
        :param int dimP: Polarity dimension of DVS sensor
        """
        super(DVS, self).__init__(net)
            
        self._xResolution = dimX
        self._yResolution = dimY
        self._pResolution = dimP

        self._numPixels = dimX*dimY*dimP
        
        self._dvsSpikeGen = DVSSpikeGen(self.net,
                                        self.xResolution,
                                        self.yResolution,
                                        self.pResolution, 
                                        0)
        
        self._dvsSpikeGen.isTestMode = isTestMode
        self._isLive = True
        self.addOutput(self._dvsSpikeGen, 'rawDVS')

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

    def setupSnips(self, board, snip=None, funcName=None, guardName=None):
        """
        Must be called. Allows a custom DVS spike injection snip to be specified. 
        Defaults to a predefined snip if none are specified.
        Either all of snip, funcName, guardName must be specified, or none.
        
        :param NxGraph board: A compiled board.
        :param string snip: Optional. Full path to the custom snip file.
        :param string funcName: Optional. The function to run.
        :param string guardName: Optional. The guard function to run.
        """
        
        #DVSSpikeGenerator(board) 
        
        if snip is None:
            snipDir = os.path.dirname(inspect.getfile(DVS)) + '/snips'
            snipFile = 'dvs_default.c'
            guardName = 'do_dvs_spike_injection'
            if self._isLive is True:
                funcName = 'dvs_live_spike_injection'
            else:
                funcName = 'dvs_host_spike_injection'
        else:
            if guardName is None or funcName is None:
                raise ValueError('<guardName> and <funcName> must '
                                 'be defined for a custom snip')
            snipDir = os.path.dirname(snip)
            snipFile = os.path.basename(snip)
            
        if self._isLive is True:  
            board.createProcess(name='runSpikes',
                                includeDir=snipDir,
                                cFilePath=snipDir + '/' + snipFile,
                                funcName=funcName,
                                guardName=guardName,
                                phase='spiking',
                                lmtId=0)
        else:
            self._dvsSpikeGen.setupHostInterface(snipDir, snipFile, funcName, guardName)

            
    def addFile(self, board, filePath, timeDuration=1000):
        """Uses an .AEDAT file as the source of spikes

        :param NxGraph board: A compiled board
        :param string filePath: The .AEDAT file to be used
        :param int timeDuration: How many .AEDAT file timesteps correspond to one Loihi timestep
        """
 
        self._dvsSpikeGen.setInputFile(board, filePath, timeDuration)
        self._isLive = False
    
    def addSpikes(self, board, time, x, y, p):
        """Wrapper for adding spikes to a dvs input
        
        :param NxGraph board: A compiled board
        :param int time: The spike times (in units of loihi timesteps)
        :param int x: The spike x addresses
        :param int y: The spike y addresses
        :param int p: The spike polarities
        """
        self._dvsSpikeGen.addSpikes(board, time, x, y, p)
        self._isLive = False