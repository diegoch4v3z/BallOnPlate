#INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
#Copyright © 2018-2021 Intel Corporation.
#
#This software and the related documents are Intel copyrighted
#materials, and your use of them is governed by the express 
#license under which they were provided to you (License). Unless
#the License provides otherwise, you may not use, modify, copy, 
#publish, distribute, disclose or transmit  this software or the
#related documents without Intel's prior written permission.
#
#This software and the related documents are provided as is, with
#no express or implied warranties, other than those that are 
#expressly stated in the License.

"""
-----------------------------------------------------------------------------
Tutorial: tutorial_21_dvs_inputfile.py
-----------------------------------------------------------------------------

This tutorial creates a DVS network and loads a pre-recorded jAER video file (.aedat)
to inject the spikes into the configured DVS network.

.. warning:: When using the DVS to feed in data from a file, the spikeGen
can be connected to other compartments/groups much like anything else. However,
when connecting to a live DVS in hardware, DVS spikes will arrive at specific
axons. Before doing anything else, the user must create a compartmentGroup to
receive these spikes and connect the DVS to it, as is done with cg1 in this tutorial.
"""
# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
import matplotlib.pyplot as plt
from nxsdk.compiler.nxsdkcompiler.n2_compiler import N2Compiler
from nxsdk_modules.dvs.src.dvs import DVS
from scipy.sparse import identity
import nxsdk.api.n2a as nx

def setupNetwork(net):
    
    # Create a DVS spike gen process
    dvs = DVS(net=net, dimX=240, dimY=180, dimP=2)

    # Create compartment prototype
    cp = nx.CompartmentPrototype(
        vThMant=100,
        enableHomeostasis=1,
        compartmentCurrentDecay=4095,
        compartmentVoltageDecay=4095,
        activityTimeConstant=0,
        activityImpulse=1,
        minActivity=20,
        maxActivity=80,
        homeostasisGain=0,
        tEpoch=1)

    # Create a compartment group
    cg1 = net.createCompartmentGroup(size=dvs.numPixels, prototype=cp)

    # Create a connection prototype
    connproto = nx.ConnectionPrototype(weight=255,
                                       signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY)

    # Create connection mask to have 1:1 mapping between pixels and compartments
    cMask = identity(dvs.numPixels)
    connGrp1 = dvs.outputs.rawDVS.connect(
        cg1, prototype=connproto, connectionMask=cMask)

    # create probes
    probeParameters = [nx.ProbeParameter.SOMA_STATE_ACTIVITY]
    probes = cg1.probe(probeParameters, None)

    compiler = N2Compiler()
    board = compiler.compile(net)

    # Set the input file
    directory = os.path.dirname(os.path.realpath(__file__))
    file = "DAVIS240C_intel.aedat"
    filePath = os.path.join(directory, file)

    # board, path to input file, 1 Loihi time step = N real time (here 1 Loihi time step = 10000 microseconds)
    dvs.addFile(board, filePath, 10000)
    dvs.setupSnips(board)

    return probes, board, dvs


if __name__ == "__main__":
    
    if 'PARTITION' not in os.environ:
        os.environ['PARTITION'] = 'nahuku32'
        
    # Create a network
    net = nx.NxNet()

    probes, board, dvs = setupNetwork(net)

    board.run(4)
    board.disconnect()

    # x, y lists for polarity 0
    x0 = list()
    y0 = list()

    # x, y lists for polarity 1
    x1 = list()
    y1 = list()

    somaProbes = probes[0]

    # soma activity shows up 2 timesteps after spike injected
    # showing plots for timestep 1 and 2 here
    for i in range(2, 4):
        for cmpt in range(0, 86400):
            if(somaProbes[cmpt].data[i] == 1):
                # convert cmpt to xaddr, yaddr, polarity
                xaddr = int(cmpt / 360)
                yaddr = int((cmpt % 360) / 2)
                polarity = int((cmpt % 360) % 2)

                if(polarity == 0):
                    x0.append(xaddr)
                    y0.append(yaddr)
                elif(polarity == 1):
                    x1.append(xaddr)
                    y1.append(yaddr)
                else:
                    print("Incorrect polarity value")
                    break

        plt.rcParams['axes.facecolor'] = 'grey'
        plt.scatter(x0, y0, color='black', s=1)
        plt.scatter(x1, y1, color='white', s=1)
        plt.xlim(0, dvs.xResolution)
        plt.ylim(0, dvs.yResolution)

        if haveDisplay:
            plt.show()
        else:
            title = "tutorial_21_fig" + str(i) + ".png"
            directory = os.getcwd()
            saveTo = os.path.join(directory, title)
            print("No display available, saving to file ", saveTo)
            plt.savefig(saveTo)
