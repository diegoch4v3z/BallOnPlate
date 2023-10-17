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

"""Benchmarks compilation of connections"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompileManyConnectionsSuite(PerfLoggingHandler):
    """Benchmarks compilation of network with 50000 connections"""

    @timeit
    @memit
    def time_workload(self, numConn=50000):
        """Times the workload"""
        net = nx.NxNet()
        p = nx.CompartmentPrototype(biasMant=100,
                                    biasExp=6,
                                    vThMant=1000,
                                    functionalState=2,
                                    compartmentCurrentDecay=409,
                                    compartmentVoltageDecay=256,
                                    logicalCoreId=0)

        c1 = net.createCompartment(p)

        p = nx.CompartmentPrototype(vThMant=1200,
                                    compartmentCurrentDecay=409,
                                    compartmentVoltageDecay=256,
                                    logicalCoreId=0)

        c2 = net.createCompartment(p)

        connProto = nx.ConnectionPrototype(weight=4)

        for ct in range(0, numConn):
            c1.connect(c2, connProto)

        c2.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                  nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                  nx.ProbeParameter.SPIKE])

        compiler = nx.N2Compiler()
        compiler.compile(net)


if __name__ == "__main__":
    CompileManyConnectionsSuite().time_workload()
