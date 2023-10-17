/*
INTEL CONFIDENTIAL

Copyright © 2020-2021 Intel Corporation.

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
#include "reset_query.h"
#include <unistd.h>
#include "static_variables.h"
#include "params.h"
#include "spiking.h"

int do_reset(runState *s) {
    // This reset runs on all cpus on all chips when the remote mgmt phase has been trigerred
    return 1;
}

void reset_cores(size_t start_core, size_t num_cores, uint16_t neurons_per_core) {
    NeuronCore* nc = NEURON_PTR(core_map[start_core]);

    CxState cxs = (CxState) {.U=0, .V=0};
    nx_fast_init_multicore(nc->cx_state, 
                           neurons_per_core, 
                           sizeof(CxState), 
                           sizeof(CxState), 
                           &cxs,
                           &core_map[start_core],
                           num_cores);     

    nx_fast_init_multicore(nc->dendrite_accum, 
                           neurons_per_core * 8192 / 1024, 
                           sizeof(DendriteAccumEntry), 
                           sizeof(DendriteAccumEntry), 
                           0,
                           &core_map[start_core],
                           num_cores);     

    MetaState ms = (MetaState) {.Phase0=2, .SomaOp0=3,
                                .Phase1=2, .SomaOp1=3,
                                .Phase2=2, .SomaOp2=3,
                                .Phase3=2, .SomaOp3=3};
    
    nx_fast_init_multicore(nc->cx_meta_state, 
                           neurons_per_core/4, 
                           sizeof(MetaState), 
                           sizeof(MetaState), 
                           &ms,
                           &core_map[start_core],
                           num_cores);
    
    for (int ii=0; ii < num_cores; ii++) {
        nx_flush_core(core_map[start_core+ii]);
    }
}


void send_timing_info() {
    uint8_t numTimingMessages = (timingIndex+31)/32;
    generalArray[0] = ((timingIndex-1)<<8) | numTimingMessages;
    writeChannel(timingChannelID, generalArray, numTimingMessages);
    timingIndex = 1;
}

void reset(runState *s) {
    
    if (isFirstCpu) {
        readChannel(spikeChannelID, spikes_in, TOKENS_PER_PACKET/32);
        spike_index = 0;
        spikeTime = spikes_in[spike_index].time;
        spikeTarget = spikes_in[spike_index].target;
    }
    
    if (s->time_step != 1) {
        if (isLastCpu) {
            uint64_t endTime = timestamp();
            msgIndex-=1;
            msg[msgIndex++].terminal_token = endTime & ((1<<16)-1);
            msg[msgIndex++].terminal_token = (endTime>>16) & ((1<<16)-1);
            msg[msgIndex++].terminal_token = (endTime>>32) & ((1<<16)-1);
            msg[msgIndex++].terminal_token = (endTime>>48) & ((1<<16)-1);

            msg[0].terminal_token = (msgIndex+31)/32; //determined by messageSize when constructing python channel
            msg[1].terminal_token = numSolutionsFound;
            writeChannel(resultChannelID, msg, msg[0].terminal_token);
        }
        #ifdef BENCHMARK
        if (isTimingCpu) send_timing_info();
        #endif
    }
    
    if (isCpu1) {
        reset_cores(NEURON_START_CORE_ROUTING, NUM_ROUTING_CORES, NEURONS_PER_CORE_ROUTING);
    }
    
    if (isCpu2) {
        reset_cores(NEURON_START_CORE_KNN, NUM_KNN_CORES, NEURONS_PER_CORE_KNN);
    }
    
    // reset the spike counters
    for (int probe_id = 0; probe_id <= STOP_COUNTER; probe_id++) {
        for(int ii=0; ii<4; ii++)
            SPIKE_COUNT[ii][0x20+probe_id] = 0;
    }
    
    if (isCpu0) {
        ack_and_drain_all_messages(numRemoteReceivePorts, remoteReceivePortArray);
        ack_and_drain_all_messages(numLocalReceivePorts, localReceivePortArray);
    }
    
    kSolutionsFound = false;
    stopped = false;
    numSolutionsFound = 0;
    newMessage = true;
    timeStart = s->time_step+1;
    msgIndex = (isLastCpu)? 2 : 0;
    startIndex = 0;
}
