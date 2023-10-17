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
# expressly stated in the License

"""Exports a graph to html given its nodes, edges and wavefront info"""

import itertools
import os

from jinja2 import Environment, FileSystemLoader

from nxsdk_modules.path_planning.src.data_model.data_model import D3ForceGraph, D3ForceGraphEdge, d3_path_planning, \
    D3ForceGraphNode
from nxsdk_modules.path_planning.src.nxcore.path_planning import PathPlanning


class ExportGraphWavefrontToHtml:
    """Exports graph wavefront to HTML Page"""

    def __init__(self, graph: PathPlanning):
        self.logger = graph.logger
        # Create a graph object
        self._g = D3ForceGraph()

        nodes, edges, src, dst, wavefront, shortestPath = graph.getWaveFront()

        internal_nodes = nodes.difference(set([src, dst]))

        edges = list(edges)

        # Add nodes
        self._g.addNode(D3ForceGraphNode(src, 1, '"src"'))
        for nodeId in internal_nodes:
            self._g.addNode(D3ForceGraphNode(nodeId, 2, nodeId))
        self._g.addNode(D3ForceGraphNode(dst, 3, '"dst"'))

        # Add edges
        for e in edges:
            self._g.addEdge(D3ForceGraphEdge(e[0], e[1]))

        # Populate a sample BFS for path planning
        num_iterations = max([step for (step, src, dst) in wavefront]) + 1
        self.logger.debug(
            "ExportGraphWavefrontToHtml: num_iterations: {}".format(num_iterations))

        node_buckets_dict = {}
        edge_buckets_dict = {}

        for (step, src, dst) in wavefront:
            if step - 1 in node_buckets_dict:
                nodeset = node_buckets_dict[step - 1]
                nodeset.add(src)
                nodeset.add(dst)
            else:
                node_buckets_dict[step - 1] = set([src, dst])

            if step - 1 in edge_buckets_dict:
                edgeset = edge_buckets_dict[step - 1]
                edgeset.add(edges.index((dst, src)))
            else:
                if src != dst:
                    edge_buckets_dict[step -
                                      1] = set([edges.index((dst, src))])
                else:
                    edge_buckets_dict[step - 1] = set([])

        # Finally add the final iteration
        node_buckets_dict[num_iterations -
                          1] = set(itertools.chain(*shortestPath))

        node_buckets = list(node_buckets_dict.values())
        self.logger.debug(
            "ExportGraphWavefrontToHtml: node_buckets: {}".format(node_buckets))

        # Finally add the final iteration
        edge_buckets_dict[num_iterations -
                          1] = set([edges.index(e) for e in shortestPath])

        edge_buckets = list(edge_buckets_dict.values())
        self.logger.debug(
            "ExportGraphWavefrontToHtml: edge_buckets: {}".format(edge_buckets))

        self._path_planning_result = d3_path_planning(
            num_iterations, node_buckets, edge_buckets)

        self._output_html = "wavefront.html"
        self._output_html_template = "wavefront.html.template"

    def _render_template(
            TEMPLATE_ENVIRONMENT,
            template_filename,
            context):
        """Given a jinja2 template renders the context"""
        return TEMPLATE_ENVIRONMENT.get_template(
            template_filename).render(context)

    def publish(self):
        """Publishes the graph wave front to html"""
        PATH = os.path.dirname(os.path.realpath(__file__))
        TEMPLATE_ENVIRONMENT = Environment(
            autoescape=False,
            loader=FileSystemLoader(os.path.join(PATH)),
            trim_blocks=False)

        context = {
            'g': self._g,
            'path_planning': self._path_planning_result,
        }

        with open(self._output_html, 'w') as f:
            html = ExportGraphWavefrontToHtml._render_template(
                TEMPLATE_ENVIRONMENT, self._output_html_template, context)
            f.write(html)
