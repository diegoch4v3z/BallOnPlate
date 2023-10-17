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

"""Runs Keyword Spotting Performance Test"""

import nxsdk
from nxsdk.logutils.benchmark_utils import timeit, memit
from nxsdk.logutils.logging_handler import PerfLoggingHandler
import os
import subprocess

from nxsdk.utils.env_var_context_manager import setEnvWithinContext


class KeywordSpottingSuite(PerfLoggingHandler):
    """Workload to benchmark ABR keyword_spotting"""

    NENGO_LOIHI_PATH = os.getcwd() + "/nengo-loihi"
    PYTHONPATH = NENGO_LOIHI_PATH + ":" + "".join(nxsdk.__path__) + "/.."
    PYTHON_SCRIPT_PATH = NENGO_LOIHI_PATH + "/docs/examples/"

    @staticmethod
    def _call(command):
        """Run a unix shell command"""
        subprocess.check_call(command, shell=True)

    def setup(self):
        """Setup the dependencies"""
        self.logger.info(
            "NENGO_LOIHI_PATH: {}".format(
                KeywordSpottingSuite.NENGO_LOIHI_PATH))
        self.logger.info(
            "PYTHONPATH: {}".format(
                KeywordSpottingSuite.PYTHONPATH))
        self.logger.info(
            "PYTHON_SCRIPT_PATH: {}".format(
                KeywordSpottingSuite.PYTHON_SCRIPT_PATH))

        # Install Nengo
        KeywordSpottingSuite._call('pip install nengo')
        # Clone the nengo-loihi repo
        KeywordSpottingSuite._call(
            'git clone https://github.com/nengo/nengo-loihi.git')
        # Convert the Keyword spotting jupyter notebook to python script
        KeywordSpottingSuite._call(
            'jupyter nbconvert --to python {}keyword_spotting.ipynb'.format(
                KeywordSpottingSuite.PYTHON_SCRIPT_PATH))

        # Replace base64 encoded import for tracecfggen due to API changes made post 0.9.0
        filePath = "{}/nengo_loihi/hardware/nxsdk_shim.py".format(KeywordSpottingSuite.NENGO_LOIHI_PATH)
        with open(filePath, 'r') as file:
            data = file.readlines()
        data[75] = '        b"bnhzZGsuYXJjaC5uMmEuY29tcGlsZXIudHJhY2VjZmdnZW4udHJhY2VjZmdnZW4=", b"VHJhY2VDZmdHZW4="'
        with open(filePath, 'w') as file:
            file.writelines(data)

    @timeit
    @memit
    def time_workload(self):
        """Run the Keyword Spotting python script"""
        KeywordSpottingSuite._call(
            'PYTHONPATH={} python '
            '{}keyword_spotting.py'.format(
                KeywordSpottingSuite.PYTHONPATH,
                KeywordSpottingSuite.PYTHON_SCRIPT_PATH))

    def teardown(self):
        """Remove all artifacts and dependencies"""
        KeywordSpottingSuite._call('pip uninstall -y nengo')
        KeywordSpottingSuite._call('rm -rf nengo-loihi')


if __name__ == '__main__':
    suite = KeywordSpottingSuite()
    try:
        suite.setup()
        suite.time_workload()
    finally:
        suite.teardown()
