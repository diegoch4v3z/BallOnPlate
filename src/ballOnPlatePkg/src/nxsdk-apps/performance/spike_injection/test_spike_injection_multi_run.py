# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

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

"""
Benchmarks compilation of network with multiple calls to addSpikes, longer spikeTimes and 4 board \
runs of 100 timesteps each
"""

import numpy as np

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class SpikeInjectionMultiRunSuite(PerfLoggingHandler):
    """Benchmarks compilation of network with multiple calls to addSpikes, longer spikeTimes and 4 board \
    runs of 100 timesteps each"""

    def setup(self):
        """Sets up the workload"""
        self.net = nx.NxNet()

        compartmentPrototype = nx.CompartmentPrototype()
        cg1 = self.net.createCompartmentGroup(
            prototype=compartmentPrototype, size=10)

        cg2 = self.net.createCompartmentGroup(
            prototype=compartmentPrototype, size=10)

        connectionPrototype = nx.ConnectionPrototype()

        self.net.createConnectionGroup(
            prototype=connectionPrototype, src=cg1, dst=cg2, connectionMask=np.eye(10))

        spikeGenerator1 = self.net.createSpikeGenProcess(numPorts=10)
        spikeTimes = []
        for i in range(10):
            j = list(range(4000))
            spikeTimes.append(j)

        spikeGenerator1.addSpikes(spikeInputPortNodeIds=list(
            range(10)), spikeTimes=spikeTimes)
        spikeGenerator1.connect(cg1, connectionPrototype)

        spikeGenerator2 = self.net.createSpikeGenProcess(numPorts=10)
        spikeTimes = []
        for i in range(10):
            j = list(range(4000))
            spikeTimes.append(j)

        spikeGenerator2.addSpikes(spikeInputPortNodeIds=list(
            range(10)), spikeTimes=spikeTimes)
        spikeGenerator2.connect(cg2, connectionPrototype)

        self.board = nx.N2Compiler().compile(self.net)

    @timeit
    def time_runNetwork(self):
        """Times the workload"""
        for i in range(4):
            self.board.run(numSteps=100)

    def teardown(self):
        """Cleanup"""
        self.board.disconnect()


if __name__ == "__main__":
    s = SpikeInjectionMultiRunSuite()
    s.setup()
    s.time_runNetwork()
    s.teardown()
