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

"""Compares benchmarks"""

import argparse
import json
import os
from collections import OrderedDict
from datetime import datetime, timedelta

from nxsdk.logutils.benchmark_encoder_decoder import BenchmarkDecoder
from nxsdk.logutils.benchmark_metrics import HigherIsBetterMetric
from performance.benchmark_constants import BENCHMARK_DIRS, JSON, PERCENTAGE
from performance.benchmark_data_model import BenchmarkDataDict, BenchmarkStat, BenchmarkImprovement, PrimaryBenchmarks
from performance.html_publisher import HTMLPublisher


class BenchmarkPlot:
    """Plots the benchmark comparison"""
    def __init__(self, title):
        self.title = title
        self.tags = []
        self.metrics = []
        self.timeseries = []

        self.xlabel = None
        self.ylabel = None
        self.sortedData = {}
        self.xdata = None
        self.ydata = None
        self.stats = None
        self.ordering = None

    def synthesize(self):
        """Synthesizes the benchmark comparison plot"""
        self.xlabel = self.metrics[0].getLabel()
        self.ylabel = self.metrics[0].getLabel()

        for tag, metric, ts in zip(self.tags, self.metrics, self.timeseries):
            self.sortedData[ts] = (tag, metric)
        self.sortedData = OrderedDict(
            sorted(self.sortedData.items(), key=lambda t: t[0]))
        self.xdata = [v[0] for v in self.sortedData.values()]
        self.ydata = [v[1].value for v in self.sortedData.values()]

        # Computes the Metrics to find change since last modification, 1 day and so on
        self.stats = self._computeStats()

        self.ordering = self.metrics[0].ordering()

    def _computeStats(self):

        stats = []
        sortedStats = list(self.sortedData.items())

        if len(sortedStats) >= 2:
            duration = "Since Last Run:"
            change = self._computePercentageChange(sortedStats[-1][1][1],
                                                   sortedStats[-2][1][1])
            stats.append(BenchmarkImprovement(duration, change, PERCENTAGE))

        for days in [1, 7, 30, 90]:
            change = self._getChangeForDays(days=days)
            if change is not None:
                duration = ("{} day:" if days ==
                            1 else "{} days:").format(days)
                stats.append(BenchmarkImprovement(
                    duration, change, PERCENTAGE))

        return stats

    def _getChangeForDays(self, days):
        """Get the change seen in the number of days"""
        d = datetime.today() - timedelta(days=days)
        filteredData = OrderedDict()
        for ts, value in self.sortedData.items():
            if ts > d:
                filteredData[ts] = value

        filteredDataList = list(filteredData.items())
        if len(filteredDataList) >= 2:
            change = self._computePercentageChange(filteredDataList[-1][1][1],
                                                   filteredDataList[0][1][1])
            return change

    def _computePercentageChange(self, new, old):
        """Compute Percentage Change between 2 Metrics"""
        result = 0.0
        if old.value != 0:
            result = round(100.0 * (old.value - new.value)/old.value, 2)
        if isinstance(new, HigherIsBetterMetric):
            result = -result
        return result

    def __repr__(self):
        return repr({"title": self.title,
                     "xdata": self.xdata,
                     "ydata": self.ydata,
                     "stats": self.stats,
                     "xlabel": self.xlabel,
                     "ylabel": self.ylabel})


class BenchmarkComparison:
    """Compares benchmarks"""
    def __init__(self, benchmarksDirectory):
        self.benchmarksDirectory = benchmarksDirectory
        self.benchmarkPlots = []

    def synthesize(self):
        """Synthesizes the benchmark comparison plots"""
        for bplot in self.benchmarkPlots:
            bplot.synthesize()

    def readBenchmarks(self):
        """Reads all recorded benchmarks from the benchmarksDirectory"""
        benchmarkRuns = []

        files = [f for f in os.listdir(
            self.benchmarksDirectory) if f.endswith(JSON)]

        for f in files:
            with open(os.path.join(self.benchmarksDirectory, f), 'r') as bf:
                b = json.load(bf, cls=BenchmarkDecoder)
                benchmarkDataDict = BenchmarkDataDict(**b)
                stats = []
                for stat in benchmarkDataDict.benchmarks:
                    stats.append(BenchmarkStat(**stat))
                benchmarkWithStats = BenchmarkDataDict(tag=benchmarkDataDict.tag,
                                                       created=benchmarkDataDict.created,
                                                       benchmarks=stats)
                benchmarkRuns.append(benchmarkWithStats)

        keyedBenchmark = OrderedDict()
        for run in benchmarkRuns:
            for b in run.benchmarks:
                for metricName in PrimaryBenchmarks:
                    metric = getattr(b, metricName)
                    if metric is not None:
                        title = "{}:{}  {}".format(
                            b.workload_group, b.workload, metricName)

                        tag = run.tag

                        created = run.created

                        if title not in keyedBenchmark:
                            keyedBenchmark[title] = BenchmarkPlot(title=title)

                        benchmarkPlot = keyedBenchmark[title]
                        benchmarkPlot.tags.append(tag)
                        benchmarkPlot.metrics.append(metric)
                        benchmarkPlot.timeseries.append(created)

        self.benchmarkPlots = keyedBenchmark.values()

    def __repr__(self):
        return repr({"benchmarksDirectory": self.benchmarksDirectory,
                     "plots": self.benchmarkPlots})

    def saveAsHTML(self):
        """Saves benchmark comparison as HTML"""
        bcp = HTMLPublisher()
        bcp.publish("benchmarks_history.html",
                    "benchmark_comparison_template.html",
                    self.benchmarkPlots,
                    [],
                    [],
                    None)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare NxSDK Benchmarks")
    parser.add_argument("-b",
                        "--benchmark-dir",
                        action="store",
                        dest="benchmarksDirectory",
                        help="path of benchmark-dir",
                        default=BENCHMARK_DIRS)

    args = parser.parse_args()

    comparator = BenchmarkComparison(args.benchmarksDirectory)
    comparator.readBenchmarks()
    comparator.synthesize()
    # print(comparator)
    comparator.saveAsHTML()
