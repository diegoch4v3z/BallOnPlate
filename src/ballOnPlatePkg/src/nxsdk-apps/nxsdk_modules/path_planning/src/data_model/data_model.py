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

"""Data objects to hold various aspects of the d3 force graph object to be passed into the jinja2 context"""

from collections import namedtuple

# Containers to hold Node and Edge.
# Every node has a id, group and name. src node has the name "src" and dst
# node has the name "dst"
D3ForceGraphNode = namedtuple('D3ForceGraphNode', 'id group name')
D3ForceGraphEdge = namedtuple('D3ForceGraphEdge', 'src dst')


class D3ForceGraph:
    """Class to contain a d3 graph with nodes and edges"""

    def __init__(self):
        self.nodes = []
        self.edges = []

    def addNode(self, n: D3ForceGraphNode):
        """Add a node"""
        self.nodes.append(n)

    def addEdge(self, e: D3ForceGraphEdge):
        """Add an edge"""
        self.edges.append(e)


# Class to hold the result of path planning. num_steps holds the number of steps needed to search
# node_buckets holds the new nodes reached in each iteration and
# edge_buckets holds the edges travered in each iteration
d3_path_planning = namedtuple(
    "d3_path_planning",
    "num_steps node_buckets edge_buckets")
