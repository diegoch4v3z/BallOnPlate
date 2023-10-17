# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2019-2021 Intel Corporation.
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

"""Workload to benchmark time for creating 2^14 compartments with 512 connections each"""

from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.benchmark_utils import memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler
from performance.workloads.network_creation.network_creation import networkCreation


class WorkloadNetworkCreationTime3Suite(PerfLoggingHandler):
    """Workload to benchmark time for creating 2^14 compartments with 512 connections each"""

    @timeit
    @memit
    def time_workload(self, numCompartments=1, numConnectionsPerCompartment=1, enableLearning=False):
        """Times the workload"""
        networkCreation(numCompartments=numCompartments,
                        numConnectionsPerCompartment=numConnectionsPerCompartment,
                        enableLearning=enableLearning)


if __name__ == '__main__':
    WorkloadNetworkCreationTime3Suite().time_workload(numCompartments=2**14,
                                                      numConnectionsPerCompartment=512,
                                                      enableLearning=False)
