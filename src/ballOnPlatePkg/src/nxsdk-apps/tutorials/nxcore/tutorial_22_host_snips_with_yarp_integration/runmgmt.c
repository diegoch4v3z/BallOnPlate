/*
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2018-2021 Intel Corporation.

This software and the related documents are Intel copyrighted
materials, and your use of them is governed by the express
license under which they were provided to you (License). Unless
the License provides otherwise, you may not use, modify, copy,
publish, distribute, disclose or transmit  this software or the
related documents without Intel's prior written permission.

This software and the related documents are provided as is, with
no express or implied warranties, other than those that are
expressly stated in the License.
*/

#include <stdlib.h>
#include <string.h>
#include "runmgmt.h"
#include <time.h>
#include <unistd.h>

int do_run_mgmt(runState *s) {
    // Runs on every timestep till the 10th timestep
    return s->time_step <= 10 ? 1 : 0;
}

void run_mgmt(runState *s) {
    // Reads an int from input channel, prints it, increments it and writes it to feedback channel
    int inputChannelID = getChannelID("input");
    int feedbackChannelID = getChannelID("feedback");

    int data[1] = {0};

    readChannel(inputChannelID, data, 1);

    printf("Input Received: %d\n", data[0]);

    // Increment data
    data[0] += 1;

    writeChannel(feedbackChannelID, data, 1);
}
