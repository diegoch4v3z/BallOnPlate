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

int spikeCountCx = 0;
int channelID = -1;
int probe_id = 0; // Represents the id of the spike_probe, e.g.: If 5 probes are
                  // created this will be 0,1,2,3,4 corresponding to the spike probes
                  // in the order in which they were created

int do_run_mgmt(runState *s) {
        if (s->time_step==1){
            channelID = getChannelID("nxspkcntr");
        }
        return 1;
}

void run_mgmt(runState *s) {
     spikeCountCx += SPIKE_COUNT[(s->time_step-1)&3][0x20+probe_id]; // This macro is used to read the lakemont spike counter
     SPIKE_COUNT[(s->time_step-1)&3][0x20+probe_id] = 0;    // Lakemont spike counters need to be cleared after reading to prevent overflow  
     writeChannel(channelID, &spikeCountCx, 1);             // Write the spike counter value back to the channel
}
