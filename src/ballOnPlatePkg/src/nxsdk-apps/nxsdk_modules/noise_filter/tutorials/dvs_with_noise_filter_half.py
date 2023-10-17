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

import nxsdk.api.n2a as nx
from nxsdk.arch.n2a.n2board import N2Board
from scipy.sparse import identity
from nxsdk.compiler.nxsdkcompiler.n2_compiler import N2Compiler
from nxsdk_modules.dvs.src.dvs import DVS
import numpy as np
import scipy.sparse as sps
import os
import errno
import subprocess
import inspect
import pickle


from nxsdk_modules.noise_filter.src.dvs_noise_filter import DVSNoiseFilter

"""
Setup a live demo using Kapoho Bay (other boards not supported) which receives live DAVIS240C data
and performs noise filtering on half the scene

Output spikes from the noise filter are visualized on the host/superhost (Kapoho Bay only)
"""

# Setup Network


def setupNetwork(net, loadState=False, boardFilename=None):
    dimX = 240
    dimY = 180
    dimP = 2
    timePerTimestep = 1e-3

    print('Creating DVS SpikeGen')

    # Create a DVS spike gen process
    dvs = DVS(net=net, dimX=dimX, dimY=dimY, dimP=dimP)

    
    # Create compartment prototype
    cp = nx.CompartmentPrototype(vThMant=100,
                                 compartmentCurrentDecay=4095,
                                 compartmentVoltageDecay=4095)

    # Create a compartment group
    cg1 = net.createCompartmentGroup(size=(dvs.numPixels), prototype=cp)

    # Create a connection prototype
    connproto = nx.ConnectionPrototype(weight=255,
                                       signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY)

    # Create connection mask to have 1:1 mapping between pixels and compartments
    cMask = identity(dvs.numPixels)
    connGrp1 = dvs.outputs.rawDVS.connect(cg1, prototype=connproto, connectionMask=cMask)

    halfSize = int(dvs.numPixels/2)

    cgright = net.createCompartmentGroup()
    for ii in range(halfSize, dvs.numPixels):
        cgright.addCompartments(cg1[ii])

    print('Setting up filter')
    # Instantiate and connect the noise filter to DVS
    filt = DVSNoiseFilter(net=net,
                          dimX=int(dimX/2), #only half size in X
                          dimY=dimY,
                          dimP=dimP,
                          timePerTimestep=timePerTimestep,
                          enableDurationTimesteps=6,
                          refractoryDurationTimesteps=6
                          )

    cgright.connect(filt.inputs.rawDVS, connectionMask=identity(halfSize))

    print('Creating output port group')
    # Create an output port group
    outPortGrp = net.createSpikeOutputPortGroup(size=dvs.numPixels)
    # and connect the DVS noise filter to it

    print('Wiring up the raw connections')
    rawOutputMask = sps.coo_matrix((np.ones(shape=(halfSize,)), (range(halfSize),
                                                                 range(halfSize))),
                                   shape=(dvs.numPixels, dvs.numPixels))

    cg1.connect(outPortGrp, connectionMask=rawOutputMask)

    print('Wiring up the filtered connections')
    filtOutputMask = sps.coo_matrix((np.ones(shape=(halfSize,)), (range(halfSize, dvs.numPixels),
                                                                  range(halfSize))),
                                    shape=(dvs.numPixels, halfSize))

    filt.outputs.filteredDVS.connect(outPortGrp, connectionMask=filtOutputMask)

    
    if loadState is False:
        print('Compiling')
        compiler = N2Compiler()
        board = compiler.compile(net)

    else:
        with open(boardFilename + '.pkl', 'rb') as fname:
            boardId, numChips, numCoresPerChip, numSynapsesPerCore = pickle.load(fname)
            
        board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    dvs.setupSnips(board)

    return filt, board


def startVisualizer():

    DVSNoiseFilterPath = os.path.dirname(inspect.getfile(DVSNoiseFilter))

    # compile the visualizer
    subprocess.run(["gcc "
                    "-O3 "
                    + "$(sdl2-config --cflags) "
                    + DVSNoiseFilterPath + "/visualizer/visualize_kb_spikes.c "
                    + "$(sdl2-config --libs) "
                    + "-o "
                    + DVSNoiseFilterPath + "/visualize_kb_spikes"],  shell=True)

    # setup spike fifo
    spikeFifoPath = DVSNoiseFilterPath + "/spikefifo"

    try:
        os.mkfifo(spikeFifoPath)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise e

    # this environment variables sets where Loihi spikes will appear on the host
    os.environ['NX_SPIKE_OUTPUT'] = spikeFifoPath

    # run the visualizer
    subprocess.Popen(
        [DVSNoiseFilterPath + "/visualize_kb_spikes", "--source=" + spikeFifoPath])

# Run Network


def runNetwork(loadState=False, boardFilename=None):

    startVisualizer()

    # Create a network
    net = nx.NxNet()

    print('Setting up Network')
    filt, board = setupNetwork(net, loadState=loadState, boardFilename=boardFilename)

    board.start()

    if loadState is False:
        if boardFilename is not None:
            print("Dumping Board")
            boardId = board.id
            numChips = len(board.n2Chips)
            numCoresPerChip = [None]*numChips
            numSynapsesPerCore = [None]*numChips
            for ii in range(numChips):
                numCoresPerChip[ii] = len(board.n2Chips[ii].n2Cores)
                numSynapsesPerCore[ii] = [None]*numCoresPerChip[ii]
                for jj in range(numCoresPerChip[ii]):
                    numSynapsesPerCore[ii][jj] = board.n2Chips[ii].n2Cores[jj].synapses.numNodes

            with open(boardFilename + '.pkl', 'wb') as fname:
                pickle.dump([boardId, numChips, numCoresPerChip, numSynapsesPerCore], fname)

                
            # Dump the NeuroCores
            board.dumpNeuroCores(boardFilename + '.board')
    else:
        print("Loading Board")
        board.loadNeuroCores(boardFilename + '.board')


    board.run(100000)
    board.disconnect()


if __name__ == "__main__":

    loadState = False
    boardFilename = "dvs_half"

    useKapohoBay = True

    if useKapohoBay is True:
        # Unset SLURM for Kapoho Bay
        os.environ['SLURM'] = ""
        # set to run on Kapoho Bay
        os.environ['KAPOHOBAY'] = '1'

    # If not set, explicitly choose a partition to ensure that the network
    # that is saved and loaded is for the same hardware
    if 'PARTITION' not in os.environ:
        os.environ['PARTITION'] = 'loihi'

    # Run the network and save the state of the board
    runNetwork(loadState=loadState, boardFilename=boardFilename)
