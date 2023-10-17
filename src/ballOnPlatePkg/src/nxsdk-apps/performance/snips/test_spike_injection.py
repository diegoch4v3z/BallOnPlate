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

"""Benchmarks channel write without packed protocol"""

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

# N2Board module provides access to the neuromorphic hardware
from nxsdk.arch.n2a.n2board import N2Board
# Input generation module used to insert a basic spike
from nxsdk.graph.nxinputgen.nxinputgen import *
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class SpikeInjectionSuite(PerfLoggingHandler):
    """Benchmarks channel write without packed protocol"""

    # Define a function to setup the network
    def setup(self, tsteps=100, numAxons=10):
        """Sets up the workload"""
        self.tsteps = tsteps
        self.numAxons = numAxons

        # -----------------------------------------------------------------------
        # Initialize board
        # -----------------------------------------------------------------------

        # Board ID
        boardId = 1

        # Number of chips
        numChips = 1

        # Number of cores per chip
        numCoresPerChip = [1]

        # Number of synapses per core
        numSynapsesPerCore = [[numAxons]]

        # Initialize the board
        self.board = N2Board(
            boardId, numChips, numCoresPerChip, numSynapsesPerCore)

        # Create a process and channel for injecting spikes
        cPath = os.path.dirname(os.path.realpath(__file__))+"/spiking.c"
        includeDir = os.path.dirname(os.path.realpath(__file__))
        funcName = "runSpiking"
        guardName = "doSpiking"
        phase = "spiking"
        spiking = self.board.createProcess(
            "spiking", cPath, includeDir, funcName, guardName, phase)
        self.spkChannel = self.board.createChannel(
            b'nxspk', "int", tsteps * 4 * numAxons * 4)
        self.spkChannel.connect(None, spiking)

        # Get the relevant core (only one in this example)
        self.board.sync = True

        # --------------------------------------------------------------------
        # Run
        # --------------------------------------------------------------------
        self.board.start()

        # Inject spikes at time steps 5, 15 and 30 to chip 0, core 4, axon 0
        # Send time, chip, core, axon for each spike
        # Send -1 at the end to indicate finished sending spike injection data
        data = []

        for t in range(tsteps):
            for axonid in range(numAxons):
                data.extend([t + 1, 0, 4, axonid])

        data.append(-1)
        self.data = tuple(data)

    @timeit
    def time_spikeChannelWrite(self):
        """Times the workload"""
        self.spkChannel.write(len(self.data), self.data)

    def teardown(self):
        """Cleanup"""
        self.board.run(self.tsteps)
        self.board.disconnect()


if __name__ == "__main__":
    suite = SpikeInjectionSuite()
    suite.setup()
    suite.time_spikeChannelWrite()
    suite.teardown()
