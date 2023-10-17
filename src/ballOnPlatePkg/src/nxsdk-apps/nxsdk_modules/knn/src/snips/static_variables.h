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

#ifndef STATIC_VARIABLES_H_
#define STATIC_VARIABLES_H_

#include "params.h"

//extern uint16_t queryNumber;

typedef struct __attribute__((packed)) {
    uint16_t target : 10;
    uint16_t time : 6;
} SpikeInput;

typedef struct __attribute__((packed)) {
        uint8_t chip;
        uint8_t timestep;
} Header;

typedef struct __attribute__((packed)) {
    uint8_t cpu : 2; 
    uint16_t index : 14;
} Spike;

typedef struct __attribute__((packed)) {
    Header header;
    Spike spike;
} IndexResult;

typedef union __attribute__((packed)) {
    uint16_t terminal_token;
    Header header;
    Spike spike;
} SpikeMessage;

extern SpikeMessage msg[K<<3];
extern ChipId max_chip_id;
extern ChipId min_chip_id;
extern ChipId my_chip_id;

extern uint16_t spike_index;

extern bool isCpu0, isCpu1, isCpu2, isFirstChip, isLastChip, isFirstCpu, isLastCpu, isTimingCpu;

extern uint8_t numRemoteReceivePorts;
extern CspRecvPort remoteReceivePortArray[2];
extern uint32_t remoteReceivePortSize;

extern uint8_t numLocalReceivePorts;
extern CspRecvPort localReceivePortArray[2];
extern uint32_t localReceivePortSize;

extern CspSendPort sendPort;

extern int resultChannelID; // Channel to send the result from the top-right most chip back to superhost post aggregtation from all other chips
extern int spikeChannelID;  // Channel to receive spikes from superhost and inject them
extern int timingChannelID; // Channel to receive per timestep timing info

extern CoreId core_map[128];

extern const int TERMINAL_TOKEN;

extern int8_t timeOffset;

extern bool kSolutionsFound; // Variable signifies that k solutions have been found on this chip using msgs passing through this chip and spikes on this chip

extern bool stopped; // Varaible signifies that CPU has stopped and should just do nothing till premption is triggered by the CPU0 of last chip

extern uint8_t manhattanHopDelay;

extern uint16_t numSolutionsFound; // This signifies number of solutions found so far on this chip using msgs passing through this chip and spikes on this chip

extern bool newMessage; // This means we are constructing a new message (all words are empty)

extern int timeStart;

extern int32_t mytime;
extern uint16_t msgIndex;
extern uint16_t startIndex;
extern uint8_t logical_cpu_id;
extern uint16_t logical_chip_id;
extern uint8_t tokensPerMessage;
extern uint16_t generalArray[GENERAL_ARRAY_SIZE];
extern uint16_t spikeTime;
extern uint16_t spikeTarget;
extern SpikeInput* spikes_in;
extern uint16_t timingIndex;
void csp_create_recv_port(ChipId chip, CoreId core, CspRecvPort *port, uint16_t size, uint8_t max);
void csp_create_send_port(ChipId chip, CoreId core, CspSendPort *port);
uint8_t csp_probe_recv(CspRecvPort *port);
volatile void *csp_peek_recv(CspRecvPort *port);
void csp_send(CspSendPort *port, const void *src);
void csp_recv(CspRecvPort *port,       void *dst);
#endif
