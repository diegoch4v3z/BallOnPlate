"""
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy,
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are
expressly stated in the License.
"""

import os
from nxsdk_modules.path_planning.src.nxcore.path_planning import PathPlanning
from nxsdk.graph.monitor.probes import *
# -----------------------------------------------------------------------------
# Run the tutorial
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Users can run the stand-alone scripts located in
    # nxsdk/compiler/standalone/graph_partition/run-utils to generate graphDirs
    # Setup the graphDir
    # python3 graphGen-nx.py -n 100000 -k 10 -t 50
    graphDir = os.path.dirname(os.path.realpath(__file__)) + "/../../data/n100000_e10"
    #graphDir = os.path.dirname(os.path.realpath(__file__)) + "/../../data/n100_e10"
    # python3 graphGen-lattice.py -d 2 -s 4 -t 1 -f
    #graphDir = os.path.dirname(os.path.realpath(__file__)) + "/../../data/2d4"

    graph = PathPlanning(graphDir)

    # Setup the network
    chipInfo, board = graph.setupNetwork()

    # Read targets
    for t in range(0, 51):
        targetList = graph.readTargets(graphDir, t)
        path = graph.searchThenTrace(chipInfo, targetList[0], targetList[1])
        if path is None:
            print("Target {}: can't trace path".format(t))
        else:
            print("Target {}: path length {}".format(t, len(path)))
    board.disconnect()
