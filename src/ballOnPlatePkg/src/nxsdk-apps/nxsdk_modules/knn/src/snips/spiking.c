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

#include "spiking.h"
#include "params.h"
#include "chip_location_helpers.h"

uint16_t spike_index = TOKENS_PER_PACKET;
uint16_t msgIndex = 0;
uint16_t startIndex = 0;
uint16_t generalArray[GENERAL_ARRAY_SIZE];
SpikeInput* spikes_in = &generalArray[0];
uint16_t spikeTime;
uint16_t spikeTarget;
uint16_t timingIndex = 1;
static int global_timestep;
SpikeMessage msg[K<<3];

int doSpiking(runState *s) {
    return 1;
}

void inject_spikes() {
    if (spikeTarget != 1022) { // 1022 means run until reset
        while (spikeTime == 0) {
            if (spikeTarget != 1023) { // 1023 means SKIP_TOKEN
                uint16_t corenum = NEURON_START_CORE_ROUTING + (spikeTarget / NEURONS_PER_CORE_ROUTING);  
                uint16_t axon = (1<<14) | (spikeTarget % NEURONS_PER_CORE_ROUTING);

                CoreId core = core_map[corenum];
                //printf("sending spike at time %d to target %d\n", global_timestep, spikeTarget);
                nx_send_remote_event(global_timestep, my_chip_id, core, axon);
            }
            spike_index++;

            spikeTime = spikes_in[spike_index].time;
            spikeTarget = spikes_in[spike_index].target; 
        }
        spikeTime--;
    }
}

void read_and_forward_messages_from_recv_ports(int numPorts, CspRecvPort* ports, int portSize, bool isLocal) {
    // Loop over remote receive channels (from other chips)
    bool receive = isLastCpu || isLocal;
    
    for (int ii=0; ii < numPorts; ii++) {
        bool endReached = false;
        uint16_t numTokens = 0;
        while (!endReached) {
            if (csp_probe_recv(&ports[ii])) {
                SpikeMessage* recPeek = csp_peek_recv(&ports[ii]);
                if (recPeek[0].header.timestep == mytime) { // only pass on data for this timestep
                    while (recPeek[(portSize>>1)-1].terminal_token != TERMINAL_TOKEN) {
                        numTokens = analyzeMsg(recPeek, portSize);      // check how many solutions in the message
                        if (receive) {
                            csp_recv(&ports[ii], &msg[msgIndex]); // receive the message
                            msgIndex += numTokens; // check how many solutions in the message
                        } else {
                            sendMsg(recPeek); // forward the message
                            csp_recv(&ports[ii], NULL);         // clear the message
                        }
                        recPeek = csp_peek_recv(&ports[ii]); //peek the next message
                    }
                    receive |= ( (recPeek[(portSize>>1)-2].terminal_token == TERMINAL_TOKEN) && (ii==numPorts-1) );
                    numTokens = analyzeMsg(recPeek, portSize);      // check how many solutions in the message
                    if (receive) {
                        csp_recv(&ports[ii], &msg[msgIndex]);       // otherwise receive the message
                        msgIndex += numTokens;
                    } else {
                        sendMsg(recPeek);         // forward the message
                        csp_recv(&ports[ii], NULL);                 // clear the message
                    }
                } else {
                    endReached = true;
                }
            } else {
                endReached = true;
            }
        }
    }
}

bool check_stopping_criterion() {
    
    if (numSolutionsFound >= K) {
        kSolutionsFound = true;
    }
    
    if (kSolutionsFound == true || SPIKE_COUNT[(global_timestep-1)&3][0x20+STOP_COUNTER] > 0) {
        stopOtherLakemonts();
        stopped = true;
    }
    return stopped;
}

void ack_and_drain_all_messages(int numPorts, CspRecvPort* ports) {
    for (int ii=0; ii < numPorts; ii++) {
        while (csp_probe_recv(&ports[ii])) {
            csp_recv(&ports[ii], NULL); // acknowledge & drain the msg as received
        }
    }
}

void read_spike_counters() {
    
    if (!stopped)
        stopped = check_stopping_criterion();
    
    mytime = global_timestep - timeStart - manhattanHopDelay - timeOffset + 2;
    
    if (stopped) {
        ack_and_drain_all_messages(numRemoteReceivePorts, remoteReceivePortArray);
        ack_and_drain_all_messages(numLocalReceivePorts, localReceivePortArray);
        return;
    }

    if (isCpu0) {
        read_and_forward_messages_from_recv_ports(numRemoteReceivePorts, remoteReceivePortArray, remoteReceivePortSize, false);
        read_and_forward_messages_from_recv_ports(numLocalReceivePorts, localReceivePortArray, localReceivePortSize, true);
    }
    
    // Now read the spike counters this spiking snip is responsible for
    msg[msgIndex].header.timestep = mytime;
    msg[msgIndex++].header.chip = logical_chip_id;
    uint16_t numSolutionsFound_pre = numSolutionsFound;
        
    uint8_t timeIdx = (global_timestep-timeOffset)&3;
    for (uint8_t sum_counter=0; sum_counter < SUM_COUNTERS; sum_counter++) {
        uint16_t probe_id = NEURONS_PER_SUM_COUNTER*sum_counter;
        while (SPIKE_COUNT[timeIdx][0x20+COUNTERS_PER_LAKEMONT+sum_counter]) {
            if (SPIKE_COUNT[timeIdx][0x20+probe_id]) {
                SPIKE_COUNT[timeIdx][0x20+COUNTERS_PER_LAKEMONT+sum_counter]--;
                SPIKE_COUNT[timeIdx][0x20+probe_id] = 0;
                //printf("counter %d activated at time %d, mytime %d\n", probe_id, global_timestep, mytime);
                msg[msgIndex].spike.index = probe_id;
                msg[msgIndex++].spike.cpu = logical_cpu_id;
                numSolutionsFound++;
            }
            probe_id++;
        }
    }
    
    // add a terminal token or delete the header
    if (numSolutionsFound_pre!=numSolutionsFound) {
        msg[msgIndex++].terminal_token = TERMINAL_TOKEN;
    } else {
        msgIndex--;
    }
    
    if (!isLastCpu) {
        while (msgIndex%tokensPerMessage)
            msg[msgIndex++].terminal_token = TERMINAL_TOKEN;

        while(startIndex!=msgIndex) {
            sendMsg(&msg[startIndex]);
            startIndex+=tokensPerMessage;
        }
    }
}

uint8_t analyzeMsg(SpikeMessage* msgIn, int msgSize) {
    for (uint8_t ii=0; ii < (msgSize >> 1); ii++) {
        if (msgIn[ii].terminal_token == TERMINAL_TOKEN) {
            if (newMessage)
                return ii;
            newMessage = true;
        } else {
            if (newMessage) {
                newMessage = false;
            }
            else {
                numSolutionsFound++;
            }
            newMessage = false;
        }
    }
    return (msgSize >> 1);
}

void sendMsg(SpikeMessage* msg) {
    if (!isLastCpu) csp_send(&sendPort, msg);
}

void stopOtherLakemonts() {
    bool chip_left = is_there_a_chip_left(my_chip_id);
    bool chip_below = is_there_a_chip_below(my_chip_id);
    
    if (isCpu0) {
        uint16_t stopAxon = (1 << 14) | (32 + STOP_COUNTER);
        nx_send_remote_event(global_timestep, my_chip_id, nx_coreid_lmt(1), stopAxon);
        nx_send_remote_event(global_timestep, my_chip_id, nx_coreid_lmt(2), stopAxon);
        
        if (chip_left)
            nx_send_remote_event(global_timestep, get_chipid_left(my_chip_id), nx_coreid_lmt(0), stopAxon);
        if (chip_below)
            nx_send_remote_event(global_timestep, get_chipid_below(my_chip_id), nx_coreid_lmt(0), stopAxon);
    }
}

void spiking(runState *s) {
    global_timestep = s->time_step;
    
    #ifdef BENCHMARK
    if (isTimingCpu) {
        generalArray[timingIndex++] = (timestamp()>>6);
        if (timingIndex==GENERAL_ARRAY_SIZE) 
            timingIndex=0;
    }
    #endif
    
    if (isFirstCpu) inject_spikes();
    read_spike_counters();
}
