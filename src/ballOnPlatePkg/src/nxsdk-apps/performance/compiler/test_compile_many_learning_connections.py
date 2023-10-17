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

"""Benchmarks compilation of connections with learning enabled"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompileManyLearningConnectionsSuite(PerfLoggingHandler):
    """Benchmarks compilation of network with 50000 connections and learning rules enabled"""

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
        plrn = nx.CompartmentPrototype(vThMant=150,
                                       compartmentCurrentDecay=3276,
                                       compartmentVoltageDecay=3276,
                                       enableSpikeBackprop=1,
                                       enableSpikeBackpropFromSelf=1,
                                       logicalCoreId=0)
        c2 = net.createCompartment(plrn)

        lr = net.createLearningRule(dw='x1*x2*y0-2*y1*y2*y3*x0',
                                    x1Impulse=40,
                                    x1TimeConstant=10,
                                    x2Impulse=43,
                                    x2TimeConstant=9,
                                    y1Impulse=42,
                                    y1TimeConstant=12,
                                    y2Impulse=41,
                                    y2TimeConstant=7,
                                    y3Impulse=39,
                                    y3TimeConstant=11,
                                    tEpoch=2)

        connProto = nx.ConnectionPrototype(weight=5, delay=0, enableLearning=1,
                                           learningRule=lr)

        for ct in range(0, numConn):
            c1.connect(c2, connProto)

        # -------------------------------------------------------------------------
        # Configure probes
        # -------------------------------------------------------------------------

        c2.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                  nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                  nx.ProbeParameter.SPIKE])

        compiler = nx.N2Compiler()
        compiler.compile(net)


if __name__ == "__main__":
    CompileManyLearningConnectionsSuite().time_workload()
