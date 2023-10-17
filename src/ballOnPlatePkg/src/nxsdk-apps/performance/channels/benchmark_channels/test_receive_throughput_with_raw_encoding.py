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

"""Benchmarking recv throughput on superhost with raw encoding"""

from nxsdk.logutils.benchmark_metrics import EmbeddedThroughput, SuperhostThroughput
from nxsdk.logutils.benchmark_utils import timeit, reportit
from performance.channels.benchmark_channels.benchmark_channels import ChannelThroughputBenchmark, ChannelType


class ReceiveThroughputWithRawEncodingSuite(ChannelThroughputBenchmark):
    """Receives unpacked (int based) data to measure recv throughput via channel"""
    @timeit
    def benchmarkChannels(self, **kwargs):
        """Benchmarks channel activity"""
        super().benchmarkChannels(**kwargs)

    @reportit
    def embeddedThroughput(self) -> EmbeddedThroughput:
        """Returns the Embedded Throughput Metric"""
        return super().embeddedThroughput()

    @reportit
    def superhostThroughput(self) -> SuperhostThroughput:
        """Returns the Superhost Throughput Metric"""
        return super().superhostThroughput()


if __name__ == '__main__':
    # Benchmarking recv channels with packed data type
    ReceiveThroughputWithRawEncodingSuite().benchmarkChannels(
        channelType=ChannelType.RECV_CHANNEL,
        elementSize=4,
        numLmts=1)