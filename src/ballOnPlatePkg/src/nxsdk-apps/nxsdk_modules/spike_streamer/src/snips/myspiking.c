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

#include "myspiking.h"
#include "nxsdk.h"
#include "streaming_header.h" 

static int time = 0;

static ChipId chip;
static CoreId core;
static uint16_t axon;
static int spike_index = 0;
static int channelID;
static bool advance_time;
static CoreId core_map[128];
static ChipId chip_map[128];
static SpikesIn spikes_in[SPIKES_PER_PACKET];
static uint64_t durationTicks;
static uint64_t deadline;

int do_spiking(runState *s) {
    return 1;
}

void run_spiking(runState *s) {

    time = s->time_step;

    if (time==1) {
        channelID = getChannelID("spikeAddresses");
        if(channelID == -1) {
              printf("Invalid channelID for spikeAddresses\n");
        }
        //prepopulate the core lookup table
        for(int ii=0; ii<128; ii++)
            core_map[ii] = nx_nth_coreid(ii);
        //prepopulate the core lookup table assumes only 32 chips
        uint8_t numChips = nx_num_chips();
        for(int ii=0; ii<numChips; ii++)
            chip_map[ii] = nx_nth_chipid(ii);
        
        //dummy as though we've just finished a packet to force reading a new packet
        spike_index = SPIKES_PER_PACKET;
        
        durationTicks = US_PER_TIMESTEP * TICKS_PER_MICROSECOND;
        deadline = timestamp();
    }

    deadline += durationTicks;
    advance_time = false; 
    while (!advance_time) // until we see the command to move to the next timestep
    {
        // if we reached the end of the packet, get another packet
        if (spike_index == SPIKES_PER_PACKET)
        {
            readChannel(channelID, &spikes_in[0], SPIKES_PER_PACKET/16);
            spike_index = 0;
        }
        
        advance_time = spikes_in[spike_index].axon == (1<<14); //the condition for advancing time
        bool skip = spikes_in[spike_index].axon == (1<<13); //the condition for skipping to the next packet
        axon = (1<<14) | spikes_in[spike_index].axon;
            
        // only inject spikes, not "advance time" messages
        if((!advance_time) & (!skip))
        {
            chip = chip_map[spikes_in[spike_index].chip];
            core = core_map[spikes_in[spike_index].core];
            //printf("Injecting Spike to t: %d chip: %d core: %d axon: %d\n", time, chip, core, axon);
            nx_send_remote_event(time, chip, core, axon);
        }
        spike_index = spike_index + 1;
    }
    
    while(timestamp() < deadline);
}

