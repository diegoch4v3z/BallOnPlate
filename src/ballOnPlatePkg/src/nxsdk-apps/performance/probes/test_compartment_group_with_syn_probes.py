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

"""
Benchmark network with 2 compartment groups of size 100 with dense connection and synapse probes
"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompartmentGroupsWithSynapseProbesSuite(PerfLoggingHandler):
    """Benchmark network with 2 compartment groups of size 100 with dense connection and synapse probes"""

    @timeit
    def time_workload(self):
        """Times the workload"""
        numCxs = 100
        numSteps = 100

        # Create a network
        net = nx.NxNet()

        prototype1 = nx.CompartmentPrototype(biasMant=100,
                                             biasExp=6,
                                             vThMant=1000,
                                             functionalState=2,
                                             compartmentVoltageDecay=256,
                                             logicalCoreId=0)
        prototype2 = nx.CompartmentPrototype(vThMant=1000, logicalCoreId=1)

        cg1 = net.createCompartmentGroup(size=numCxs, prototype=prototype1)
        cg2 = net.createCompartmentGroup(size=numCxs, prototype=prototype2)
        connProto = nx.ConnectionPrototype(
            weight=4, signMode=nx.SYNAPSE_SIGN_MODE.EXCITATORY)
        conns = cg1.connect(dstGrp=cg2, prototype=connProto)

        probeConditions = None
        connProbeParameters = [nx.ProbeParameter.SYNAPSE_WEIGHT,
                               nx.ProbeParameter.SYNAPSE_TAG,
                               nx.ProbeParameter.SYNAPSE_DELAY]
        conns.probe(connProbeParameters, probeConditions)

        compiler = nx.N2Compiler()
        board = compiler.compile(net)

        board.run(numSteps)

        board.disconnect()


if __name__ == '__main__':
    CompartmentGroupsWithSynapseProbesSuite().time_workload()
