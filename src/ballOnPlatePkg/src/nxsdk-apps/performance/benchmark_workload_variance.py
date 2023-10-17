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
import argparse
from typing import Sequence

from nxsdk.logutils.logging_handler import PerfLoggingHandler
from performance.benchmark_comparison import BenchmarkComparison, BenchmarkPlot
from performance.benchmark_constants import BENCHMARK_DIRS, WORKLOAD_VARIANCE_PERCENTAGE_THRESHOLD, PERCENTAGE


class WorkloadVarianceChecker(PerfLoggingHandler):
    """Checks the workload runs for variance and degradation below threshold level"""

    def __init__(self, threshold: float, workloads: Sequence[BenchmarkPlot]):
        super().__init__()
        self.logger.info(
            msg="Checking for workload variance. Threshold set to {}{}".format(
                threshold, PERCENTAGE))
        self._threshold = threshold
        self._workloads = workloads

    def check(self):
        errors = []

        for workload in self._workloads:
            if workload.stats:
                last_run_variance = workload.stats[0].change
                if last_run_variance < 0 and abs(
                        last_run_variance) > self._threshold:
                    msg = "Workload: {} degradation from prior run {}{}".format(
                        workload.title, last_run_variance, PERCENTAGE)
                    errors.append(msg)

        if errors:
            list(map(self.logger.error, errors))
            log_msg = "\n" + "\n".join(errors)
            raise Exception(log_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Detects variance between current and last benchmark runs")
    parser.add_argument(
        "-t",
        "--threshold",
        action="store",
        dest="threshold",
        help="threshold beyond which the current workload will be flagged as slower than previous run",
        default=WORKLOAD_VARIANCE_PERCENTAGE_THRESHOLD)
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

    vc = WorkloadVarianceChecker(threshold=float(args.threshold) * 100,
                                 workloads=comparator.benchmarkPlots)
    vc.check()
