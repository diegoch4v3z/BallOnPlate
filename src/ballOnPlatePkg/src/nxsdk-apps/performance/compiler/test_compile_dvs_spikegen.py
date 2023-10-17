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

"""Benchmarks DVS SpikeGen"""

from scipy.sparse import identity

import nxsdk.api.n2a as nx
from nxsdk.compiler.nxsdkcompiler.n2_compiler import N2Compiler
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompileDVSSpikeGenSuite(PerfLoggingHandler):
    """Benchmarks compilation of a DVS Spike Gen Process with resolution 240*180*2
    and 1:1 mapping to a compartment group"""

    @timeit
    @memit
    def time_workload(self):
        """Times the workload"""
        # Create a network
        net = nx.NxNet()

        # Create a dvs spike generator
        dvsSpikeGen = net.createDVSSpikeGenProcess(
            xPixel=240, yPixel=180, polarity=2)
        dvsSpikeGen.isTestMode = True

        # Create compartment prototype
        cp = nx.CompartmentPrototype()

        # Create a compartment group consisting of xPixel*yPixel*polarity compartments having cp prototytpe
        cg1 = net.createCompartmentGroup(
            size=dvsSpikeGen.numPorts, prototype=cp)

        # Create a connection prototype
        connproto = nx.ConnectionPrototype(
            weight=1, numWeightBits=-1, signMode=2)

        # creating a connection mask to have 1:1 mapping between pixels and compartment
        cMask = identity(dvsSpikeGen.numPorts)
        dvsSpikeGen.connect(cg1, prototype=connproto, connectionMask=cMask)

        compiler = N2Compiler()
        compiler.compile(net)


if __name__ == "__main__":
    CompileDVSSpikeGenSuite().time_workload()
