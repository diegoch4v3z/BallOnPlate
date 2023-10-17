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

"""Benchmarks compilation of compartments"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompileManyCompartmentsSuite(PerfLoggingHandler):
    """Benchmarks compilation of network with 100000 compartments"""

    @timeit
    @memit
    def time_workload(self, numCompartments=100000):
        """Times the workload"""
        net = nx.NxNet()

        p = nx.CompartmentPrototype(biasMant=100,
                                    biasExp=6,
                                    vThMant=1000,
                                    functionalState=2,
                                    compartmentCurrentDecay=409,
                                    compartmentVoltageDecay=256)
        for ct in range(0, numCompartments):
            net.createCompartment(p)

        compiler = nx.N2Compiler()
        compiler.compile(net)


if __name__ == "__main__":
    CompileManyCompartmentsSuite().time_workload()
