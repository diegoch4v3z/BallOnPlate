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

"""
Benchmarks a run on 100 compartments without any probes
"""

# Nx API
import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompartmentsWithoutProbesSuite(PerfLoggingHandler):
    """Benchmarks a run on 100 compartments without any probes"""

    @timeit
    def time_workload(self, numCxs=100, numSteps=100):
        """Times the workload"""
        net = nx.NxNet()

        prototype1 = nx.CompartmentPrototype(biasMant=100,
                                             biasExp=6,
                                             vThMant=1000,
                                             functionalState=2,
                                             compartmentVoltageDecay=256,
                                             logicalCoreId=0)

        compartment1 = net.createCompartment(prototype1)

        for logicalCore in range(1, numCxs):
            compartmentPrototype = nx.CompartmentPrototype(
                vThMant=1000, logicalCoreId=0)
            compartment = net.createCompartment(compartmentPrototype)
            connectionPrototype = nx.ConnectionPrototype(weight=64)
            compartment1.connect(compartment, connectionPrototype)

        compiler = nx.N2Compiler()

        board = compiler.compile(net)

        board.run(numSteps)

        board.disconnect()


if __name__ == '__main__':
    CompartmentsWithoutProbesSuite().time_workload()
