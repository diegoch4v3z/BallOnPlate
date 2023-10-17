# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2018-2021 Intel Corporation.

# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.

# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

"""Benchmark channel read write every 20 time steps"""

import os
import timeit as builtin_timeit

from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.logutils.benchmark_metrics import AverageTimeStepMetric
from nxsdk.logutils.benchmark_utils import reportit, timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class ChannelReadWritePer20TimeStepSuite(PerfLoggingHandler):
    """Benchmark channel read write every 20 time steps"""

    def setup(self, SKIP):
        """Sets up the benchmark"""
        self.SKIP = SKIP

        self.board = N2Board(1, 1, [1], [[10]])

        cPath = os.path.dirname(os.path.realpath(__file__)) + "/initsnip.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "initA"
        guardName = None
        phase = "init"
        initProcess = self.board.createProcess("initProcess",
                                               cPath,
                                               includeDir,
                                               funcName,
                                               guardName,
                                               phase)

        # Creating another snip to do management whenever domgmt condition is true
        cPath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt_20.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "runMgmt"
        guardName = "doRunMgmt"
        phase = "mgmt"
        self.board.createProcess("runMgmt", cPath, includeDir,
                                 funcName, guardName, phase)

        self.imageChannel = self.board.createChannel(
            b'imageChannel', "int", 15)
        self.recvChannel = self.board.createChannel(b'recvChannel', "int", 15)

        self.imageChannel.connect(None, initProcess)
        self.recvChannel.connect(initProcess, None)
        self.board.start()

        # Creating data to be send
        self.data = []
        for i in range(15000):
            self.data.append(i)

    @timeit
    @reportit
    def time_writeChannel(self):
        """Times the workload"""
        # Writing 5000 elements on the imageChannel
        self.imageChannel.write(15, self.data)

        start = builtin_timeit.default_timer()

        n_steps = 100

        # Run the execution for 1 cycle
        self.board.run(n_steps, aSync=True)
        for i in range(1, n_steps+1):
            if i % self.SKIP == 0:
                datarecv = self.recvChannel.read(15)
                self.imageChannel.write(15, self.data[i*15:i*15+15])

        end = builtin_timeit.default_timer()
        avgTimeStepInMilliseconds = round((end-start)*1000/n_steps, 2)

        # Receiving the data back from the host. In the initProcess snip
        # we are incrementing the data received by 4 and sending it back.
        datarecv = self.recvChannel.read(15)
        return AverageTimeStepMetric(avgTimeStepInMilliseconds)

    def teardown(self):
        """Cleanup"""
        self.board.disconnect()


if __name__ == "__main__":
    suite = ChannelReadWritePer20TimeStepSuite()
    suite.setup(SKIP=20)
    suite.time_writeChannel()
    suite.teardown()
