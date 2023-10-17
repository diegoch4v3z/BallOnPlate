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

"""Publishing for the benchmark"""

import json
import os
from abc import ABC
from datetime import datetime

from nxsdk.logutils.benchmark_encoder_decoder import BenchmarkEncoder
from nxsdk.logutils.benchmark_utils import phaseToFunctionLookup
from nxsdk.logutils.logging_handler import PerfLoggingHandler
from performance.benchmark_constants import HTML, JSON, BENCHMARK_DIRS
from performance.benchmark_data_model import BenchmarkDataDict, MetricsBreakdownOrderedDictKeys, \
    MetricsBreakdownUnits
from performance.html_publisher import HTMLPublisher


class AbstractBenchmarkPublisher(ABC):
    """Defines publish API given filename, benchmarks to be captured and any extra args"""

    def publish(self, filename, benchmarks, **kwargs):
        """Abstract function to do the publishing"""
        pass


class HTMLBenchmarkPublisher(AbstractBenchmarkPublisher):
    """Publishes benchmarks as HTML"""

    def publish(self, filename, benchmarks, **kwargs):
        """Function to do the publishing"""
        htmlPublisher = HTMLPublisher()
        htmlPublisher.publish(filename,
                              "benchmark_template.html",
                              benchmarks,
                              MetricsBreakdownOrderedDictKeys,
                              MetricsBreakdownUnits,
                              phaseToFunctionLookup)


class JsonBenchmarkPublisher(AbstractBenchmarkPublisher, PerfLoggingHandler):

    """Publishes benchmarks as JSON"""

    def publish(self, tag, benchmarks, **kwargs):
        """Function to do the publishing"""
        try:
            self.logger.warn("Creating {} directory".format(BENCHMARK_DIRS))
            os.mkdir(BENCHMARK_DIRS)
        except FileExistsError as _:    # pylint:disable=E0602, E0603
            self.logger.warn(
                "Saving to existing {} directory".format(BENCHMARK_DIRS))
        except Exception as e:
            raise e

        filename = "{}.json".format(tag)

        benchmarksData = []
        for b in benchmarks:
            benchmarksData.append(b._asdict())

        data = BenchmarkDataDict(tag, datetime.now(), benchmarksData)

        with open(os.path.join(BENCHMARK_DIRS, filename), 'w') as f:
            json.dump(data._asdict(), f, cls=BenchmarkEncoder, indent=4)


def getBenchmarkPublisher(exportType):
    """Factory method to create benchmark publisher"""
    if exportType == HTML:
        return HTMLBenchmarkPublisher()
    elif exportType == JSON:
        return JsonBenchmarkPublisher()
    else:
        raise RuntimeError(
            "Invalid publisher. Supported options are html and json")
