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
-----------------------------------------------------------------------------
Tutorial: tutorial_25_sequential_host_snips.py
-----------------------------------------------------------------------------

This tutorial demonstrates sequential host snips. Sequential host snips execute
either in the pre-execution phase (before the network executes on chip
un-interrupted) or the post-execution phase. When the network needs to sync
back to the host, it will cause delays so for the most performance critical
code, custom logic should be written as embedded snips and executed on lakemont
(embedded processor). However, when certain requirements like larger code/memory
sizes, memory handling, external communication or third party linkages are
necessary, host snips can be used to execute the custom code.

In this tutorial, we have 10 compartments with 10 input axons. Once the network
is configured, we run for 110 steps but in strides of 10. The input axon ids for
spike injection are read from a file by the pre-execution snip. For the first 10
steps, each compartment is injected spike in a round robin fashion. Upon finishing
10 steps, the post execution snip kicks in and changes the distribution to a normal
distribution with mean varying from 0 to 9 for steps of 10. The post execution snip
only kicks at the last timestep of the run. An embedded snip runs on every
timestep and injects the spike event to the input axon.

This tutorial also demonstrates the ability of host snips to read a file.
"""

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

import matplotlib.colors as mcolors
import numpy as np

from nxsdk.graph.processes.phase_enums import Phase
import nxsdk.api.n2a as nx
import matplotlib.pyplot as plt
import atexit

from nxsdk.logutils.nxlogging import set_verbosity, LoggingLevel
set_verbosity(LoggingLevel.WARNING)

modified_path = None
precomputed_axon_file_name = os.path.dirname(
    os.path.realpath(__file__)) + "/precomputed_axons"


def write_precomputed_axons_file(net, logicalAxonIds):
    """Writes the physical axons ids to a file so that it can be read by the host snip"""

    # Once network is compiled, we can fetch the physical axon ids and write it to the
    # pre-computed axons file to be read by the host snip

    physicalAxonIds = []
    for logicalAxonId in logicalAxonIds:
        _, _, _, physicalAxonId = net.resourceMap.inputAxon(logicalAxonId)[0]
        physicalAxonIds.append(physicalAxonId)

    # Write physicalAxonIds to the precomputed_axons file
    with open(precomputed_axon_file_name, "w") as f:
        f.write("\n".join(str(axon) for axon in physicalAxonIds))


def update_host_snip_with_path_to_precomputed_axons_file(filePath):
    """Update the filePath with the runtime path for the precomputed_axons file"""
    global modified_path
    with open(filePath, 'r') as file:
        data = file.readlines()

    for index, line in enumerate(data):
        if 'static std::string precomputed_axons_file' in line:
            data[index] = 'static std::string precomputed_axons_file = "{}";\n'.format(
                precomputed_axon_file_name)

    tokens = os.path.split(filePath)
    modified_fileName = "modified_" + tokens[-1]

    modified_path = os.path.join(tokens[0], modified_fileName)

    with open(modified_path, 'w') as file:
        file.writelines(data)

    atexit.register(cleanup)
    return modified_path


def cleanup():
    """Delete the generated snip file"""
    if modified_path:
        os.remove(modified_path)
        os.remove(precomputed_axon_file_name)


def setupNetwork(net):
    """Creates the network, snips and connections"""

    NUM_COMPARTMENTS = 10

    # Create a compartment group with a low threshold so that each spike
    # injected makes the compartment spike
    prototype = nx.CompartmentPrototype()
    prototype.compartmentThreshold = 64
    cg = net.createCompartmentGroup(prototype=prototype, size=NUM_COMPARTMENTS)

    # Attach spike probes to the compartment group
    spikeProbes = cg.probe(nx.ProbeParameter.SPIKE)[0]

    # spikeInputPorts creates input axons for each of the compartments.
    # These input axon IDs will be stored in a file which the pre-execution
    # host snip reads. The input axon IDs are used within the embedded snip to
    # inject spikes to associated compartments.
    spikeInputPorts = net.createSpikeInputPortGroup(size=NUM_COMPARTMENTS)
    connProto = nx.ConnectionPrototype(weight=127)
    connGrp = spikeInputPorts.connect(
        dstGrp=cg,
        prototype=connProto,
        connectionMask=np.eye(NUM_COMPARTMENTS))

    # Get the logical Input Axon Ids
    logicalAxonIds = []
    for inputPort in spikeInputPorts:
        logicalAxonIds.append(inputPort.inputAxons[0].nodeId)

    # Compile the network
    board = nx.N2Compiler().compile(net)

    # Convert logical to physical axon IDs and wite the physical axon IDs to a
    # file to be read by the host snip
    write_precomputed_axons_file(net, logicalAxonIds)

    # Creating embedded snip to do spiking whenever doPhase condition is true
    cFilePath = os.path.dirname(
        os.path.realpath(__file__)) + "/embedded_snip.c"
    includeDir = os.path.dirname(os.path.realpath(__file__))
    funcName = "runPhase"
    guardName = "doPhase"

    embeddedProcess = board.createSnip(phase=Phase.EMBEDDED_SPIKING,
                                       cFilePath=cFilePath,
                                       includeDir=includeDir,
                                       funcName=funcName,
                                       guardName=guardName)

    # Create pre and post execution host snips. All classes are defined in the same file but they can
    # be put in different files. Classes registered in the same phase in the same file are stored
    # in the snip registry in order. But this is not guarenteed across files. So, if you split
    # the implementation and they inter-dependencies, try to keep the classes which belong together
    # in the same translation unit (cppfile).
    cppFile = os.path.dirname(os.path.realpath(__file__)) + "/host_snip.cc"
    cppFile = update_host_snip_with_path_to_precomputed_axons_file(cppFile)
    selectAxonForSpikeInjection = board.createSnip(
        phase=Phase.HOST_PRE_EXECUTION, cppFile=cppFile)
    changeSpikeDistribution = board.createSnip(
        phase=Phase.HOST_POST_EXECUTION, cppFile=cppFile)

    # Create a channel named axon_info for selecting input axon to inject spike
    axon_selection = board.createChannel(
        name=b'axon_info',
        messageSize=4,
        numElements=100)
    # Connecting axon_selection channel from Host process to embedded process
    # The host process (selectAxonForSpikeInjection) selects the input axon and the embeddedProcess
    # sends the spike event to the input axon
    axon_selection.connect(selectAxonForSpikeInjection, embeddedProcess)

    return board, spikeProbes


if __name__ == "__main__":

    # Create an instance of NxNet
    net = nx.NxNet()

    # Set up the network
    board, sprobes = setupNetwork(net)

    for i in range(11):
        board.run(10)

    board.disconnect()

    fig = plt.figure(1)

    # Plot the spike probes
    sprobes.plot(colors=list(mcolors.TABLEAU_COLORS.values()))

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_25_fig1.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)
