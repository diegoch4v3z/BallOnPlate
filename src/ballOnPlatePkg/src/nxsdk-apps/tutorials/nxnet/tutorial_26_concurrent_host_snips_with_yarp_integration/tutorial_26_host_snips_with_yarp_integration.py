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

"""
This tutorial demonstrates using yarp ports with host snips.

Pre-requisites:

1. Install yarp. Follow: http://www.yarp.it/install_yarp_linux.html
    1.a For Xenial: sudo sh -c 'echo "deb http://www.icub.org/ubuntu xenial contrib/science" > /etc/apt/sources.list.d/icub.list'
    1.b sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 57A5ACB6110576A6
    1.c sudo apt-get update
    1.d sudo apt-get install yarp

2. Start yarpserver by running: yarpserver
    Ensure it comes up

3. On your machine, ensure you have CMake to build the host snip as we need to create the shared library with linkages
   to yarp libs

Tutorial is intended to currently run on Kapohobay based setup. Minor modifications to CMakeLists.txt and yarp setup
would be required in other scenarios.

Scenario:

Tutorial code is adapted from http://www.yarp.it/yarp_cmake_hello.html

In this tutorial, we create the following pipeline:

HostProcess (HostSnip) --------------->input channel---------------> EmbeddedSnip runSpiking (read from channel)
 reads /cx/in                                                              |
        ^                                                                  |
        |                                                                  V
        |                                                                Update
        |                                                                  |
        |                                                                  |
 writes /cx/out                                                            V
HostProcess (HostSnip) <---------------feedback channel<------------- EmbeddedSnip runSpiking (write to channel)

In the Host Snip, we connect the yarp output and input ports
yarp.connect(outPort->getName(), inPort->getName());

In thsi tutorial, we create a compartment group of size N and connect it using spikegen. Then using concurrent host
snip(HostProcess) pick a cx in round robin format, write it to ouputPort. Using yarp inputPort we read the cxId and
send spike to it by sending the id to EmbeddedProcess using NxSDK channels. From the embedded snip, monitor which cx
spiked, send the address of it along with the time stamp to the HostProcess.

Please note: HostProcess is a concurrent snip running in parallel with neurocore execution.
"""

import os
import subprocess
import numpy as np
import matplotlib as mpl

haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

import matplotlib.pyplot as plt
import nxsdk.api.n2a as nx
from nxsdk.graph.nxprobes import SpikeProbeCondition
from nxsdk.graph.processes.phase_enums import Phase


RUN_FOR_TIME_STEPS = 14


def build_shared_libary() -> str:
    """Build the host snip shared library"""
    lib = os.path.dirname(os.path.realpath(__file__)) + \
        "/build/libyarp_host_snip.so"
    # Compile the Host Snip to create the library after linking with yarp libs
    build_script = "{}/build.sh".format(
        os.path.dirname(os.path.realpath(__file__)))
    subprocess.call([build_script], shell=True, executable="/bin/bash")
    return lib


def setup_network(shared_library_path: str, N=4):
    # Creating the NxNet object
    net = nx.NxNet()

    # Creating compartment group of size N
    # CxGroup has no bias current and is driven by spikes
    # received

    cxPrototype = nx.CompartmentPrototype(biasMant=0,
                                          biasExp=0,
                                          vThMant=10,
                                          functionalState=2,
                                          compartmentVoltageDecay=0,
                                          compartmentCurrentDecay=4096)

    compartmentGrp = net.createCompartmentGroup(prototype=cxPrototype, size=N)

    # Creating Spike Input port to create input axons and connect it to
    # compartment Group.

    spikeGen = net.createSpikeInputPortGroup(size=N)
    connProto = nx.ConnectionPrototype(weight=12)
    connGrp = spikeGen.connect(
        compartmentGrp,
        prototype=connProto,
        connectionMask=np.eye(N))

    # Hacking spike probes to create spike counter and defer probing
    # Check tutorial on lakemont spike counters
    # lmt counters created will be read from Embedded snip
    probeParameters = [
        nx.ProbeParameter.SPIKE]
    pc = SpikeProbeCondition(dt=1, tStart=10000000)
    spikeProbes = compartmentGrp.probe(probeParameters, pc)

    # Creating volatge probe to monitor Voltage activity of
    # members of the compartmentGroup
    voltageProbe = compartmentGrp.probe(
        [nx.ProbeParameter.COMPARTMENT_VOLTAGE])
    board = net.compiler.compile(net)

    # Create the host snip and assign it to run in Concurrent Execution phase
    hostProcess = board.createSnip(
        phase=Phase.HOST_CONCURRENT_EXECUTION,
        library=shared_library_path)

    # Creating snip to run in spiking phase
    # This snip inserts spike in axonId received via the channel, which
    # is send from the host.
    cFilePath = os.path.dirname(os.path.realpath(__file__)) + "/spiking.c"
    includeDir = os.path.dirname(os.path.realpath(__file__))
    funcName = "runSpiking"
    guardName = "doSpiking"
    embeddedProcess = board.createSnip(
        phase=Phase.EMBEDDED_SPIKING,
        cFilePath=cFilePath,
        includeDir=includeDir,
        funcName=funcName,
        guardName=guardName)

    # Creating a channel named input for sending data from host snip
    # to embedded snip (spiking phase)
    inputChannel = board.createChannel('input', "int", 1000)
    # Connecting input channel from inputProcess to embeddedProcess making it
    # send channel
    inputChannel.connect(hostProcess, embeddedProcess)

    # Create a channel named feedback for getting the feedback value from the
    # embedded snip
    feedbackChannel = board.createChannel('feedback', "int", 1000)
    # Connecting feedback channel from embeddedProcess to outputProcess making
    # it receive channel
    feedbackChannel.connect(embeddedProcess, hostProcess)

    return board, voltageProbe


if __name__ == "__main__":
    # Build the shared library for the host architecture
    shared_library_path = build_shared_libary()
    # Setup the network, host snips and embedded snip
    board, voltageProbe = setup_network(shared_library_path)

    # Run for RUN_FOR_TIME_STEPS based on the scenario described above
    # The embedded snip receives spikes on  from host snip, prints, and sends the adrress of cx spiked
    # in that timestep back to the host along with the timestamp.
    # The host then increments the cxId and send spike to next cx.
    # The yarp output port is connected to input port in the host snip.
    # cxId of the cx to which spike is to be sent is written on output port.
    # cxId of the cx to which spike is to be sent is read from input port.
    
    try:
        board.run(RUN_FOR_TIME_STEPS)
    finally:
        board.disconnect()

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------

    fig = plt.figure(1)
    plt.subplot(1, 4, 1)
    voltageProbe[0][0].plot()
    plt.subplot(1, 4, 2)
    voltageProbe[0][1].plot()
    plt.subplot(1, 4, 3)
    voltageProbe[0][2].plot()
    plt.subplot(1, 4, 4)
    voltageProbe[0][3].plot()

    # plot the voltage of cxs
    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_26_fig1.png"
        print("No display available, saving to file " + fileName1)
        fig.savefig(fileName1)
