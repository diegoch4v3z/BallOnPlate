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

"""Benchmark for 100 compartment with bias creation, compilation, and running"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class WorkloadComp3Suite(PerfLoggingHandler):
    """Workload to benchmark a network with 100 compartments bias on, no probes"""

    @timeit
    @memit
    def time_workload(self):
        """Wrapper to time the workload"""
        # Create a network
        net = nx.NxNet()

        p = nx.CompartmentPrototype(
            biasExp=6, biasMant=100, vThMant=1000, functionalState=2)
        net.createCompartmentGroup(size=100, prototype=p)

        net.run(100)
        net.disconnect()


if __name__ == '__main__':
    WorkloadComp3Suite().time_workload()
