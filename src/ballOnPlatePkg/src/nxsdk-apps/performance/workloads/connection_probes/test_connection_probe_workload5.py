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

"""Workload to benchmark a network of 200 compartments, 10000 connections, all synapse probes"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler

class WorkloadConnectionProbe5Suite(PerfLoggingHandler):
    """Workload to benchmark a network of 200 compartments, 10000 connections, all synapse probes"""
    @timeit
    @memit
    def time_workload(self):
        """Times the workload"""
        net = nx.NxNet()
        pbasic = nx.CompartmentPrototype(vThMant=150,
                                         compartmentCurrentDecay=3276,
                                         compartmentVoltageDecay=3276,
                                         logicalCoreId=0)

        connProto1 = nx.ConnectionPrototype(weight=200, delay=0)

        cg1 = net.createCompartmentGroup(size=100, prototype=pbasic)
        cg2 = net.createCompartmentGroup(size=100, prototype=pbasic)

        conn = cg1.connect(cg2, prototype=connProto1)
        conn.probe([nx.ProbeParameter.SYNAPSE_WEIGHT,
                    nx.ProbeParameter.SYNAPSE_DELAY, nx.ProbeParameter.SYNAPSE_TAG], None)
        net.run(100)
        net.disconnect()


if __name__ == '__main__':
    WorkloadConnectionProbe5Suite().time_workload()
