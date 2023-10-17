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
    1.a For Ubuntu Focal: sudo sh -c 'echo "deb http://www.icub.org/ubuntu focal contrib/science" > /etc/apt/sources.list.d/icub.list'
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

InputProcess (HostSnip) --------------->input channel---------------> EmbeddedSnip runMgmt (read from channel)
 reads /hello/in                                                           |
        ^                                                                  |
        |                                                                  V
        |                                                                Update
        |                                                                  |
        |                                                                  |
 writes /hello/out                                                         V
FeedbackProcess (HostSnip) <---------------feedback channel<------------- EmbeddedSnip runMgmt (write to channel)

In the Host Snip, we connect the yarp output and input ports
yarp.connect(outPort->getName(), inPort->getName());

InputProcess writes an integer into the input channel. EmbeddedSnip reads it and increments the same.
It then writes the incremented value to the feedback channel. Feedback process reads this integer.
Feedback process then writes this integer onto yarp /hello/out. InputProcess receives the data by reading yarp /hello/in

Please note: InputProcess and FeedbackProcess execute on every timestep. InputProcess is a PreExecution Snip
while FeedbackProcess is a PostExecutionSnip
"""

import os
import subprocess
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.processes.phase_enums import Phase


def build_shared_libary() -> str:
    """Build the host snip shared library"""
    lib = os.path.dirname(os.path.realpath(__file__)) + "/build/libyarp_host_snip.so"
    # Compile the Host Snip to create the library after linking with yarp libs
    build_script = "{}/build.sh".format(os.path.dirname(os.path.realpath(__file__)))
    subprocess.call([build_script], shell=True, executable="/bin/bash")
    return lib


def setup_network(shared_library_path: str):
    """Setup the Network as mentioned in the scenario"""
    # Instantiate the N2Board
    board = N2Board(1, 1, [1], [[10]])

    # Create the host snips
    inputProcess = board.createSnip(phase=Phase.HOST_PRE_EXECUTION, library=shared_library_path)
    outputProcess = board.createSnip(phase=Phase.HOST_POST_EXECUTION, library=shared_library_path)

    # Creating another snip to do management whenever do_run_mgmt condition is true
    cFilePath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c"
    includeDir = os.path.dirname(os.path.realpath(__file__))
    funcName = "run_mgmt"
    guardName = "do_run_mgmt"
    embeddedProcess = board.createSnip(phase=Phase.EMBEDDED_MGMT, cFilePath=cFilePath, includeDir=includeDir,
                                            funcName=funcName, guardName=guardName)

    # Create a channel named input for sending data from host snip (pre execution) to embedded snip (mgmt phase)
    inputChannel = board.createChannel(b'input', "int", 30)
    # Connecting input channel from inputProcess to embeddedProcess making it send channel
    inputChannel.connect(inputProcess, embeddedProcess)

    # Create a channel named feedback for getting the feedback value from the embedded snip
    feedbackChannel = board.createChannel(b'feedback', "int", 30)
    # Connecting feedback channel from embeddedProcess to outputProcess making it receive channel
    feedbackChannel.connect(embeddedProcess, outputProcess)

    return board


if __name__ == "__main__":
    # Build the shared library for the host architecture
    shared_library_path = build_shared_libary()
    # Setup the network, host snips and embedded snip
    board = setup_network(shared_library_path)

    # Run for 10 steps based on the scenario described above
    # The embedded snip receives tokens 0 to 9 from host snip, prints, incrememts and sends it back to host
    # The post execution host snip communicates to the pre execution host snip via yarp ports to complete the loop
    try:
        board.run(10)
    finally:
        board.disconnect()
