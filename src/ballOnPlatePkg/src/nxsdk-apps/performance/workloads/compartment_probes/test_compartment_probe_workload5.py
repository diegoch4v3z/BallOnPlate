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

"""Benchmark for probing voltage and current"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class WorkloadCompProbes5Suite(PerfLoggingHandler):
    """Workload to benchmark a network with 1024 compartments, bias on, all probes"""

    @timeit
    @memit
    def time_workload(self):
        """Times the workload"""
        # Create a network
        net = nx.NxNet()

        p = nx.CompartmentPrototype(
            biasExp=6, biasMant=100, vThMant=1000, functionalState=2)
        cg = net.createCompartmentGroup(size=1024, prototype=p)
        cg.probe([nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                  nx.ProbeParameter.COMPARTMENT_CURRENT, nx.ProbeParameter.SPIKE], None)
        net.run(100)
        net.disconnect()


if __name__ == '__main__':
    WorkloadCompProbes5Suite().time_workload()
