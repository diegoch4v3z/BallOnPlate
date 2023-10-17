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
Benchmark network with 2 compartment groups of size 100 with dense connection and \
compartment probes
"""

import nxsdk.api.n2a as nx
from nxsdk.logutils.benchmark_utils import timeit
from nxsdk.logutils.logging_handler import PerfLoggingHandler


class CompartmentGroupsWithRegisterProbesSuite(PerfLoggingHandler):
    """Benchmark network with 2 compartment groups of size 100 with dense connection and \
    compartment probes"""

    @timeit
    def time_workload(self, numCxs=100, numSteps=100):
        """Times the workload"""
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
        cxProbeParameters = [nx.ProbeParameter.COMPARTMENT_CURRENT,
                             nx.ProbeParameter.COMPARTMENT_VOLTAGE,
                             nx.ProbeParameter.SPIKE]

        # Create a compartment probe to probe the states of each compartment as
        # specified by the probeParameters and probeConditions
        cg1.probe(cxProbeParameters, probeConditions)
        cg2.probe(cxProbeParameters, probeConditions)

        compiler = nx.N2Compiler()
        board = compiler.compile(net)

        board.run(numSteps)
        board.disconnect()


if __name__ == '__main__':
    CompartmentGroupsWithRegisterProbesSuite().time_workload()
