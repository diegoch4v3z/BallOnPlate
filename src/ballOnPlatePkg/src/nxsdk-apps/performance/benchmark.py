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

"""The NxSDK benchmark tools"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time
from collections import OrderedDict

from nxsdk.logutils.benchmark_encoder_decoder import BenchmarkEncoder, BenchmarkDecoder
from nxsdk.logutils.benchmark_metrics import Accumulable
from nxsdk.logutils.logging_handler import PerfLoggingHandler
from performance.benchmark_data_model import BenchmarkStat, MetricsBreakdownOrderedDictKeys
from performance.benchmark_publisher import HTMLBenchmarkPublisher, JsonBenchmarkPublisher


class BenchmarkSuiteOutputProcessor:
    """Process a suite output"""
    @staticmethod
    def process(suite, stdoutLines, logger):
        """Processes stdout to extract benchmarks"""
        splitlines = stdoutLines.decode('ascii').splitlines()

        workload = None
        stdout = []
        description = "Description is not available"
        workload_group = ".".join(suite.split("/")[:-1])
        runtime = None
        memoryUse = None
        metricsBreakdown = OrderedDict.fromkeys(
            MetricsBreakdownOrderedDictKeys)

        for line in splitlines:
            stdout.append(line)
            tokens = line.split(":")
            if len(tokens) >= 3:
                if tokens[2].strip() in MetricsBreakdownOrderedDictKeys:
                    metricAsJsonString = ":".join(tokens[4:])
                    metric = json.loads(metricAsJsonString,
                                        cls=BenchmarkDecoder)
                    step = tokens[2].strip()
                    if metricsBreakdown[step]:
                        if isinstance(metric, Accumulable):
                            metricsBreakdown[step].value = round(
                                metricsBreakdown[step].value + metric.value, 4)
                        else:
                            logger.warn("Overwriting {} with {} as metric is not Accumulable".
                                        format(metricsBreakdown[step], metric))
                            metricsBreakdown[step] = metric
                    else:
                        metricsBreakdown[step] = metric
                elif line.startswith("INFO:PRF:") and "Suite" in line and "TimeMetric" in line:
                    workload = tokens[3].split('.')[0]
                    metricAsJsonString = ":".join(tokens[4:])
                    runtime = json.loads(
                        metricAsJsonString, cls=BenchmarkDecoder)
                elif line.startswith("INFO:PRF:") and "Suite" in line and "MemoryMetric" in line:
                    workload = tokens[3].split('.')[0]
                    metricAsJsonString = ":".join(tokens[4:])
                    memoryUse = json.loads(
                        metricAsJsonString, cls=BenchmarkDecoder)
                elif line.startswith("INFO:PRF:") and "DESC:" in line:
                    description = " ".join(tokens[3:])

        stat = BenchmarkStat(workload, description, workload_group,
                             runtime, memoryUse, metricsBreakdown, "\n".join(stdout))

        return stat


class Benchmark(PerfLoggingHandler):
    """Runs benchmark and all discovered suites"""

    def __init__(self, globPattern="test_*py"):
        super().__init__()
        self.logger.info(msg="Running Benchmark Suite")
        self.globPattern = globPattern
        self.discovered_benchmark_suites = []
        self.baseDir = os.path.dirname(os.path.realpath(__file__))

        self.parentDirectory = "{}/..".format(self.baseDir)
        self.logger.info(
            "Running benchmark from {}".format(self.parentDirectory))

        self.benchmarkOutput = []
        self.errors = 0

    def run(self):
        """Runs the Benchmark suite after discovering all eligible performance tests"""
        cwd = os.getcwd()
        os.chdir(self.baseDir)
        try:
            globPattern = "**/{}".format(self.globPattern)
            self.discovered_benchmark_suites = sorted(
                glob.glob(globPattern, recursive=True))
            for suite in self.discovered_benchmark_suites:
                self.runSuite(suite)
        finally:
            os.chdir(cwd)

    def raiseSysExitOnFailure(self):
        """If any benchmark fails, exit with error status to signal performance build failure"""
        if self.errors:
            self.logger.error(
                "Performance Run Failed. Number of Workload Errors: {}".format(self.errors))
            sys.exit(1)

    def runSuite(self, suite):
        """Runs a single performance test"""
        self.logger.info("Running suite {}".format(suite))
        command = "python3 -m performance/{}".format(
            suite).replace("/", ".").replace(".py", "")
        try:
            p = subprocess.run(command.split(), stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, check=True, cwd=self.parentDirectory,
                               env=dict(os.environ, NXSDK_ENABLE_STATS="1", NXSDK_DISABLE_ENERGY_STATS="1"))
            self.saveRunOutput(suite, p.stdout)
        except subprocess.CalledProcessError as calledProcessError:
            self.errors += 1
            self.logger.error("Failed to run suite {}. Exception {}".format(
                suite, calledProcessError.output.decode('ascii')))
        except Exception as e:
            self.errors += 1
            self.logger.error(
                "Failed to run suite {}. Exception {}".format(suite, str(e)))

    def saveRunOutput(self, suite, stdout):
        """Append the benchmark output to the previous benchmarks"""
        suiteProcessedResult = BenchmarkSuiteOutputProcessor.process(
            suite, stdout, self.logger)
        self.benchmarkOutput.append(suiteProcessedResult)

    def _formatStr(self):
        stringList = []
        for b in self.benchmarkOutput:
            s = json.dumps(b._asdict(), cls=BenchmarkEncoder)
            stringList.append(s)
        return "\n".join(stringList)

    def __repr__(self):
        return self._formatStr()

    def saveAsHTML(self, path):
        """Save the benchmark as HTML"""
        bp = HTMLBenchmarkPublisher()
        bp.publish(path, self.benchmarkOutput)

    def saveAsJson(self, tag):
        """Save the benchmark as Json"""
        bp = JsonBenchmarkPublisher()
        bp.publish(tag, self.benchmarkOutput)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Run the NxSDK Benchmark Suite")
    parser.add_argument("-p",
                        "--pattern",
                        action="store",
                        dest="pattern",
                        help="glob pattern to be selected")

    parser.add_argument("-t",
                        "--tag",
                        action="store",
                        dest="tag",
                        help="benchmark will be tagged with this field for comparison")

    args = parser.parse_args()

    tag = int(time.time()) if args.tag is None else args.tag

    if args.pattern:
        benchmark = Benchmark(args.pattern)
    else:
        benchmark = Benchmark()

    # Run the benchmark suite
    benchmark.run()
    # print(benchmark)

    # Save the result of current run as html
    benchmark.saveAsHTML("benchmarks.html")

    # Tag and Save the result of current run as json
    benchmark.saveAsJson(tag)

    # Raise error on failures so that the build fails if any test fails.
    benchmark.raiseSysExitOnFailure()
