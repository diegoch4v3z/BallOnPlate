/*
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

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

static int count = 0;
static int channelID = -1;
static int32_t spike_counts[16*NUM_PACKED] = {0}; //assumes 16 or fewer output classes for now

int do_run_mgmt(runState *s) {
        
        if (s->time_step==1){
            channelID = getChannelID("nxspkcntr");
        }
    
        if ((count == TIMESTEPS_PER_SAMPLE-1) || (count%512 == 0)) {
            return 1; //read the spike counters
        }else{
            count = count + 1;
            return 0; //otherwise do nothing this time
        }
}

void run_mgmt(runState *s) {

        for(int probe_id = 0; probe_id<NUM_OUTPUTS; probe_id++) {
                //------- to probe on every timestep, use this:
                //spike_counts[probe_id] += SPIKE_COUNT[(s->time_step-1)&3][0x20+probe_id]; 
                //SPIKE_COUNT[(s->time_step-1)&3][0x20+probe_id] = 0;
            
                //but rather probe occasionally and use this:
                for(int ii=0; ii<4; ii++){
                    spike_counts[probe_id] += SPIKE_COUNT[ii][0x20+probe_id]; 
                    SPIKE_COUNT[ii][0x20+probe_id] = 0;
                }
        }

        if (count == TIMESTEPS_PER_SAMPLE-1) {
            count = 0;
            // Write the spike counter value back to the channel and reset our spike counts
            writeChannel(channelID, spike_counts, NUM_PACKED);
            for(int probe_id = 0; probe_id<NUM_OUTPUTS; probe_id++)
                spike_counts[probe_id] = 0;
        }else{
            count = count + 1;
        }
}
