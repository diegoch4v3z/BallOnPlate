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

"""Defines benchmark names and units"""

from collections import namedtuple

"""Stores the primary benchmarks for plotting"""
PrimaryBenchmarks = ["runtime", "memory_use"]

"""Stores statistics associated with a particular file, classInstance and method"""
BenchmarkStat = namedtuple("BenchmarkStat", "workload description workload_group " +
                           ' '.join(PrimaryBenchmarks) + " metrics stdout")

"""Stores a collection of benchmarks given tag and created"""
BenchmarkDataDict = namedtuple("BenchmarkDataDict", "tag created benchmarks")

"""Represents improvement given a duration type, change and percentage(%)"""
BenchmarkImprovement = namedtuple(
    "BenchmarkImprovement", "duration change percentage")

"""All metrics which will be parsed from stdout"""
MetricsBreakdownOrderedDictKeys = ("COMPILE", "A0", "A1Ext", "A1.1", "A1.2",
                                   "A1.3.1", "A1.3.2", "A1.3.3", "A1.4",
                                   "A2", "A2.1", "A2.2",
                                   "B1", "B2", "B4",
                                   "C1.1", "C1.2", "C2", "C4", "C5",
                                   "A3",
                                   "E1.1", "E1.2",
                                   "D2.1", "D2.2", "D2.3", "D2.4", "D2.5", "D2",
                                   "G1", "G1.1", "G1.2", "G1.3", "G1.4", "G1.5",
                                   "P1", "P1.1", "P1.2", "P1.3", "P1.4",
                                   "Q1", "Q2")
MetricsBreakdownUnits = ("s", "s", "s", "s", "s", "s", "s", "s", "s", "s",
                         "s", "s",
                         "s", "s", "s",
                         "s", "s", "s", "s", "s",
                         "s",
                         "s", "s",
                         "us", "us", "us", "us", "us", "us",
                         "uJ", "uJ", "uJ", "uJ", "uJ", "uJ",
                         "mW", "mW", "mW", "mW", "mW",
                         "s", "s")

if len(MetricsBreakdownOrderedDictKeys) != len(MetricsBreakdownUnits):
    raise Exception(
        "Length of MetricsBreakdownOrderedDictKeys and MetricsBreakdownUnits do not match")
