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
#include "spiking.h"
#include "nxsdk.h"

static int time=0;
static int chip;
static int core;
static int axon;

int do_spiking(runState *s) {
    return 1;
}

void run_spiking(runState *s) {
     // convert logical chip id and logical axon id to physical ids
    int channelID = getChannelID("nxspiking");
    time = s->time_step;
    readChannel(channelID, &chip, 1);
    readChannel(channelID, &core, 1);
    readChannel(channelID, &axon, 1);
    uint16_t  axonId = 1<<14 | (axon & 0x3FFF);
    ChipId    chipId = nx_nth_chipid(chip);
    printf("Sending Spike at Time : %d chip %d core %d axon %d\n",s->time_step,chip,core,axon);

    // send the spike
    nx_send_remote_event(time, chipId, (CoreId){.id=core}, axonId);
}
