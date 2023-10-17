# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2018-2021 Intel Corporation.
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


# Nx API
import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class BasicSpikeInjectionSuite(PerfLoggingHandler):
    """Benchmarks spike injection using addSpikes and network run"""

    @timeit
    def time_workload(self, numSteps=100, numCompartments=512):
        """Times the workload"""
        # Create a network
        net = nx.NxNet()

        prototype = nx.CompartmentPrototype(biasMant=100,
                                            biasExp=6,
                                            vThMant=1000,
                                            functionalState=2,
                                            compartmentVoltageDecay=256)

        cg = net.createCompartmentGroup(
            prototype=prototype, size=numCompartments)

        connProto1 = nx.ConnectionPrototype(weight=1, signMode=2)

        # Spike Generator
        numPorts = 1
        spikeGen = net.createSpikeGenProcess(numPorts)
        spikeGen.connect(cg, prototype=connProto1)

        spikeTimes = list(range(1, numSteps+1))
        spikeGen.addSpikes(0, spikeTimes)

        compiler = nx.N2Compiler()
        board = compiler.compile(net)

        board.run(numSteps)
        board.disconnect()


if __name__ == "__main__":
    BasicSpikeInjectionSuite().time_workload()
