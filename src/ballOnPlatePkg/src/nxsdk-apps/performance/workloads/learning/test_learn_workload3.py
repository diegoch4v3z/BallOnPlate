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

"""Workload to benchmark a network of 200 compartments, 1000 connections, learning on(1)"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class WorkloadLearn3Suite(PerfLoggingHandler):
    """Workload to benchmark a network of 200 compartments, 1000 connections, learning on(1)"""
    @timeit
    @memit
    def time_workload(self):
        """Times the workload"""
        net = nx.NxNet()
        plrn = nx.CompartmentPrototype(vThMant=150,
                                       compartmentCurrentDecay=3276,
                                       compartmentVoltageDecay=3276,
                                       enableSpikeBackprop=1,
                                       enableSpikeBackpropFromSelf=1,
                                       logicalCoreId=0)

        lr = net.createLearningRule(dw='2*x1*y0-2*y1*x0',
                                    x1Impulse=40,
                                    x1TimeConstant=4,
                                    y1Impulse=40,
                                    y1TimeConstant=4,
                                    tEpoch=1)

        connProto1 = nx.ConnectionPrototype(
            weight=50, delay=0, enableLearning=1, learningRule=lr)

        cg1 = net.createCompartmentGroup(size=100, prototype=plrn)
        cg2 = net.createCompartmentGroup(size=100, prototype=plrn)

        for j in range(0, 10):
            for i in range(0, 100):
                cg1[i].connect(cg2[j], prototype=connProto1)

        net.run(100)
        net.disconnect()


if __name__ == '__main__':
    WorkloadLearn3Suite().time_workload()
