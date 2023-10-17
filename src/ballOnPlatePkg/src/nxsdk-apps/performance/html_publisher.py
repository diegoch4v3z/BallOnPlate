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

"""Creates HTML from benchmark results"""

import datetime
import os
from jinja2 import Environment, FileSystemLoader

class HTMLPublisher:
    """Class for creating HTML from benchmark results"""
    def _render_template(self, TEMPLATE_ENVIRONMENT, template_filename, context):
        return TEMPLATE_ENVIRONMENT.get_template(template_filename).render(context)

    def publish(self, filename, jinja2_template, benchmarks, breakdownMetrics, breakdownMetricUnits, phaseToFunctionLookup, **kwargs):
        """Publishes the benchmark results to HTML file"""
        PATH = os.path.dirname(os.path.realpath(__file__))
        TEMPLATE_ENVIRONMENT = Environment(
            autoescape=False,
            loader=FileSystemLoader(os.path.join(PATH, 'templates')),
            trim_blocks=False)

        context = {
            'benchmarks': benchmarks,
            'breakdownMetricsHeaders': zip(breakdownMetrics, breakdownMetricUnits),
            'phaseToFunctionLookup': phaseToFunctionLookup,
            'snapshotTime': datetime.datetime.now()
        }

        with open(filename, 'w') as f:
            html = self._render_template(TEMPLATE_ENVIRONMENT,
                                         jinja2_template,
                                         context)
            f.write(html)
