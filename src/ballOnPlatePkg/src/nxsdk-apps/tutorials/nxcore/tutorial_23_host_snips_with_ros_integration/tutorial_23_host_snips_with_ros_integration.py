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
This tutorial demonstrates using ROS publisher/subscriber nodes with host snips.

Pre-requisites:

1. Install ROS. Follow: ROS_INSTALLATION_AND_SETUP.md

2. Start roscore by running: source /opt/ros/noetic/setup.bash; roscore
    Ensure it comes up (This will block the terminal - so you need another terminal to run this tutorial)

3. source /opt/ros/noetic/setup.bash

4. On your machine, ensure you have CMake to build the host snip as we need to create the shared library with linkages
   to ros libs

Tutorial is intended to currently run on Kapohobay based setup. Minor modifications to CMakeLists.txt and ros setup
would be required in other scenarios.

Scenario:

Tutorial code is adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

In this tutorial, we create the following pipeline:

PubSubProcess (HostSnip) --------------->input channel---------------> EmbeddedSnip runMgmt (read from channel)
 subscribes to TOPIC example                                               |
        ^                                                                  |
        |                                                                  V
        |                                                                Update
        |                                                                  |
        |                                                                  |
 publishes to TOPIC example                                                V
PubSubProcess (HostSnip) <---------------feedback channel<------------- EmbeddedSnip runMgmt (write to channel)

We create a concurrent host snip with class PubSubProcess which implements both subscriber and publisher.

PubSubProcess writes an integer into the input channel. EmbeddedSnip reads it and increments the same.
It then writes the incremented value to the feedback channel. PubSubProcess reads this integer.
_pub within PubSubProcess then publishes this to ros topic "example". _sub within PubSubProcess receives the data
by subscribing to the same topic
"""

import os
import subprocess
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.processes.phase_enums import Phase


def build_shared_libary() -> str:
    """Build the host snip shared library"""
    lib = os.path.dirname(os.path.realpath(__file__)) + \
        "/build/libros_host_snip.so"

    # Compile the Host Snip to create the library after linking with ros libs
    build_script = "{}/build.sh".format(
        os.path.dirname(os.path.realpath(__file__)))
    subprocess.run(
        [build_script],
        check=True,
        shell=True,
        executable="/bin/bash")

    return lib


def setup_network(shared_library_path: str):
    """Setup the Network as mentioned in the scenario"""
    # Instantiate the N2Board
    board = N2Board(1, 1, [1], [[10]])

    # Create the host snip
    pubSubProcess = board.createSnip(
        phase=Phase.HOST_CONCURRENT_EXECUTION,
        library=shared_library_path)

    # Creating another snip to do management whenever do_run_mgmt condition is
    # true
    cFilePath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c"
    includeDir = os.path.dirname(os.path.realpath(__file__))
    funcName = "run_mgmt"
    guardName = "do_run_mgmt"
    embeddedProcess = board.createSnip(
        phase=Phase.EMBEDDED_MGMT,
        cFilePath=cFilePath,
        includeDir=includeDir,
        funcName=funcName,
        guardName=guardName)

    # Create a channel named input for sending data from host snip (pre
    # execution) to embedded snip (mgmt phase)
    inputChannel = board.createChannel(b'input', "int", 1000000)
    # Connecting input channel from pubSubProcess to embeddedProcess making it
    # send channel
    inputChannel.connect(pubSubProcess, embeddedProcess)

    # Create a channel named feedback for getting the feedback value from the
    # embedded snip
    feedbackChannel = board.createChannel(b'feedback', "int", 1000000)
    # Connecting feedback channel from embeddedProcess to pubSubProcess making
    # it receive channel
    feedbackChannel.connect(embeddedProcess, pubSubProcess)

    return board


if __name__ == "__main__":
    # Build the shared library for the host architecture
    shared_library_path = build_shared_libary()
    # Setup the network, host snips and embedded snip
    board = setup_network(shared_library_path=shared_library_path)

    # Run for 1000 steps based on the scenario described above
    # The embedded snip receives tokens 0 to 999 from host snip, prints, incrememts and sends it back to host
    # The concurrent host snip publishes to the ros topic. Upon receiving a new message, subscriber callback is invoked
    # to complete the loop
    try:
        board.run(1000)
    finally:
        board.disconnect()
