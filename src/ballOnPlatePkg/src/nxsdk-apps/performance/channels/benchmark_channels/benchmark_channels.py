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

"""Benchmark suite that reports channel throughput for Embedded and Superhost"""

from nxsdk.logutils.logging_handler import PerfLoggingHandler
from nxsdk.logutils.benchmark_metrics import EmbeddedThroughput, \
    SuperhostThroughput
from nxsdk.arch.n2a.n2board import N2Board
from enum import IntEnum
from jinja2 import Environment, FileSystemLoader
import sys
import argparse
import time
import os
from nxsdk.logutils.nxlogging import timedContextLogging, NxSDKLogger


class ChannelType(IntEnum):
    """Enum class to represent send and receive channels"""
    SEND_CHANNEL = 0
    RECV_CHANNEL = 1


class ChannelThroughputBenchmark(PerfLoggingHandler):
    """Benchmark suites that reports channel throughput for Embedded and Superhost"""

    def benchmarkChannels(self,
                          channelType=ChannelType.SEND_CHANNEL,
                          elementSize=64,
                          numChips=1,
                          numLmts=1,
                          dataSize=2**20,
                          slack=16):
        """
        Benchamrks the Channel Throughput
        :param channelType: Type of the Channel : Send, Recv
        :param elementSize: Size of the packet
        :param numChips: Num of chips on which these channels will be replicated
        :param numLmts: Num of lmts for a chip on which these channels will be replicated
        :param dataSize: Size of the data to be send/recv via the channel
        :return: Superhost average throughput, Lmt average throughput
        """
        msg = "Benchmarking Channel Activity: " \
              "ChannelType: {} DataType: {} NumChips: {} NumLmts: {} DataSize: {} Slack: {}".format(
                  channelType.name, elementSize, numChips, numLmts, dataSize, slack)

        with timedContextLogging(msg, NxSDKLogger.PERF, indent=0):
            # Create board with n2Chip and n2Core
            board = N2Board(1, numChips, [1] * numChips, [[5]] * numChips)

            includeDir = os.path.dirname(os.path.realpath(__file__))
            funcName = "runSpiking"
            guardName = "doSpiking"
            phase = "spiking"

            writeChannels = []
            readChannels = []
            lmtChannels = []
            dataSizeChannels = []
            infoChannels = []

            files = []

            for chip in range(numChips):
                for lmt in range(numLmts):
                    # Creating snip to perform channel_operation data
                    if channelType == ChannelType.SEND_CHANNEL:
                        cPath = os.path.dirname(os.path.realpath(
                            __file__)) + "/spiking_read_{}_{}.c".format(chip, lmt)
                        headerFile = os.path.dirname(os.path.realpath(
                            __file__)) + "/spiking_read_{}_{}.h".format(chip, lmt)
                        templateFile = "spiking_read.c.template"
                    else:
                        cPath = os.path.dirname(os.path.realpath(
                            __file__)) + "/spiking_write_{}_{}.c".format(chip, lmt)
                        headerFile = os.path.dirname(os.path.realpath(
                            __file__)) + "/spiking_write_{}_{}.h".format(chip, lmt)
                        templateFile = "spiking_write.c.template"

                    headerTemplateFile = "spiking.h.template"

                    context = {
                        "chip": chip,
                        "lmt": lmt
                    }

                    env = Environment(
                        loader=FileSystemLoader(
                            os.path.dirname(
                                os.path.realpath(__file__)) +
                            "/templates"),
                        trim_blocks=True)

                    template = env.get_template(templateFile)
                    spikeFile = template.render(context)
                    with open(cPath, 'w') as snip:
                        snip.write(spikeFile)

                    template = env.get_template(headerTemplateFile)
                    header = template.render()
                    with open(headerFile, 'w') as hFile:
                        hFile.write(header)

                    files.extend([cPath, headerFile])

                    runSpiking = board.createProcess(
                        "runSpiking",
                        cPath,
                        includeDir,
                        funcName,
                        guardName,
                        phase,
                        chipId=chip,
                        lmtId=lmt)

                    # Create channel named imageChannel for sending data
                    spikeChannel = board.createChannel(
                        'spikeChannel_{}_{}'.format(
                            chip,
                            lmt),
                        messageSize=elementSize,
                        numElements=dataSize,
                        slack=slack)
                    numElements = dataSize // (elementSize // 4)

                    infoChannel = board.createChannel(
                        'infoChannel_{}_{}'.format(
                            chip, lmt), messageSize=4, numElements=1)
                    infoChannel.connect(None, runSpiking)
                    infoChannels.append(infoChannel)

                    lmtResultChannel = board.createChannel(
                        'lmtResultChannel_{}_{}'.format(
                            chip, lmt), messageSize=4, numElements=2)
                    lmtResultChannel.connect(runSpiking, None)
                    lmtChannels.append(lmtResultChannel)

                    dataSizeChannel = board.createChannel(
                        'dataSizeChannel_{}_{}'.format(
                            chip, lmt), messageSize=4, numElements=1)
                    dataSizeChannel.connect(None, runSpiking)
                    dataSizeChannels.append(dataSizeChannel)

                    # Connecting the channel
                    if channelType == ChannelType.SEND_CHANNEL:
                        spikeChannel.connect(None, runSpiking)
                        writeChannels.append(spikeChannel)
                    else:
                        spikeChannel.connect(runSpiking, None)
                        readChannels.append(spikeChannel)

            # Synthetic data to be written
            writeData = list(range(dataSize))

            try:
                # Start the board
                board.start()
                for channel in dataSizeChannels:
                    channel.write(1, [dataSize])
                for channel in infoChannels:
                    channel.write(1, [elementSize])
                if channelType == ChannelType.SEND_CHANNEL:
                    aggregate = 0
                    for writeChannel in writeChannels:
                        t1 = time.time()
                        writeChannel.write(numElements, writeData)
                        t2 = time.time()
                        superhostResult = (t2 - t1)
                        aggregate += (dataSize * 32) / \
                            (superhostResult * (2**20))
                    self.average = aggregate / len(writeChannels)
                    board.run(1)
                else:
                    aggregate = 0
                    board.run(1)
                    for readChannel in readChannels:
                        t3 = time.time()
                        data = readChannel.read(numElements)
                        t4 = time.time()
                        superhostResult = (t4 - t3)
                        aggregate += (dataSize * 32) / \
                            (superhostResult * (2 ** 20))
                    self.average = aggregate / len(readChannels)
                self.logger.info(
                    "Superhost : Throughput is : {} Mb/s".format(self.average))

                lmtAggregate = 0
                for lmtChannel in lmtChannels:
                    result = lmtChannel.read(2)
                    lmtresult = result[0] / result[1]
                    lmtAggregate += (dataSize * 32) / (lmtresult * (2**20))
                self.lmtAverage = lmtAggregate / len(lmtChannels)
                self.logger.info(
                    "Embedded : Throughput is : {} Mb/s".format(self.lmtAverage))
            finally:
                board.disconnect()
                for f in files:
                    os.remove(f)

            self.superhostThroughput()
            self.embeddedThroughput()

    def embeddedThroughput(self) -> EmbeddedThroughput:
        """Returns the Embedded Throughput Metric"""
        return EmbeddedThroughput(round(self.lmtAverage, 4))

    def superhostThroughput(self) -> SuperhostThroughput:
        """Returns the Superhost Throughput Metric"""
        return SuperhostThroughput(round(self.average, 4))


if __name__ == "__main__":
    if len(sys.argv) > 1:
        parser = argparse.ArgumentParser(
            description='Calculates Read and Write Throughput of the Channel on Lmt and Superhost side')
        parser.add_argument(
            '--channel-type', '-c',
            default='0',
            type=int,
            help='Channel Type : SEND (0) or RECV (1)')
        parser.add_argument(
            '--element-size', '-e',
            default='0',
            type=int,
            help='Element Size : Needs to be multiple of 4')
        parser.add_argument(
            '--data-size', '-d',
            default='1048576',
            type=int,
            help='Data Size : size of the data to be send (numElements)')
        parser.add_argument(
            '--num-chips', '-n',
            default='1',
            type=int,
            help='Number of Chips on which channels will be replicated')
        parser.add_argument(
            '--num-lmts', '-l',
            default='1',
            type=int,
            help='Number of Lmts per chip on which channels will be replicated')
        parser.add_argument(
            '--slack', '-s',
            default='16',
            type=int,
            help='Slack of the channel')

        args = parser.parse_args()

        ChannelThroughputBenchmark().benchmarkChannels(
            channelType=ChannelType(args.channel_type),
            elementSize=args.element_size,
            dataSize=args.data_size,
            numChips=args.num_chips,
            numLmts=args.num_lmts,
            slack=args.slack)
