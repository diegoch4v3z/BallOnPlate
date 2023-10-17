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

#include "reconfigure.h"
#include "channel_setup.h"

ChipId max_chip_id;
ChipId min_chip_id;
ChipId my_chip_id;

uint8_t numRemoteReceivePorts;
CspRecvPort remoteReceivePortArray[2];
uint32_t remoteReceivePortSize = REMOTE_MESSAGE_BYTE_SIZE;

uint8_t numLocalReceivePorts;
CspRecvPort localReceivePortArray[2];
uint32_t localReceivePortSize = LOCAL_MESSAGE_BYTE_SIZE;

CspSendPort sendPort;
int resultChannelID = -1;  // Channel to send the result from the top-right most chip back to superhost post aggregtation from all other chips
int spikeChannelID = -1; // Channel to receive spikes from superhost and inject them
int timingChannelID = -1;
CoreId core_map[128];

int8_t timeOffset;

bool kSolutionsFound; // Variable signifies that k solutions have been found on this chip using msgs passing through this chip and spikes on this chip

bool stopped; // Varaible signifies that CPU has stopped and should just do nothing till premption is triggered by the CPU0 of last chip

uint8_t manhattanHopDelay;

uint16_t numSolutionsFound; // This signifies number of solutions found so far on this chip using msgs passing through this chip and spikes on this chip

bool newMessage = true; // This means we are constructing a new message (all words are empty)

int timeStart;

int32_t mytime;

uint8_t logical_cpu_id;
uint16_t logical_chip_id;

uint8_t tokensPerMessage;

bool isCpu0, isCpu1, isCpu2, isFirstChip, isLastChip, isFirstCpu, isLastCpu, isTimingCpu;
const int TERMINAL_TOKEN = (1<<16)-1;

//corenum specifies which core to configure
//routingCoreNum specifies which knn cores this is
void setupKnnCore(int corenum, int knnCoreNum) {
    
    // setup general core parameters
    NeuronCore* knnCore = NEURON_PTR(nx_nth_coreid(corenum));
    
    knnCore->synapse_fmt[1].WgtBits = 7;
    knnCore->synapse_fmt[1].NumSynapses = NEURONS_PER_CORE_KNN;
    knnCore->synapse_fmt[1].IdxBits = 0;
    knnCore->synapse_fmt[1].Compression = 3;
    knnCore->synapse_fmt[1].FanoutType = 1;
    
    knnCore->synapse_fmt[2].WgtExp = 7;
    knnCore->synapse_fmt[2].WgtBits = 7;
    knnCore->synapse_fmt[2].NumSynapses = 63;
    knnCore->synapse_fmt[2].IdxBits = 5;
    knnCore->synapse_fmt[2].FanoutType = 1;
    
    knnCore->dendrite_accum_cfg.DelayBits = 0;
    
    knnCore->dendrite_shared_cfg.PosVmLimit = 7;
    knnCore->dendrite_shared_cfg.NegVmLimit = 23;
    
    knnCore->cx_profile_cfg[0].bAP_Action = 1;
    knnCore->cx_profile_cfg[0].RefractDelay = 63;
    
    knnCore->vth_profile_cfg[0].vthProfileStaticCfg.Vth = THRESHOLD_KNN;
    knnCore->num_updates.num_updates = (NEURONS_PER_CORE_KNN+3)/4;

    // setup input axons for all:all input to KNN routing
    DiscreteMapEntry synMapEntry;
    synMapEntry.CxBase = 0;
    synMapEntry.Len = SYN_MEM_LENGTH; //20 bits for header, 8 bits per synapse, 64 bits per word
    synMapEntry.Ptr = 0;
    for (int ii=0; ii<NEURONS_PER_CHIP_ROUTING; ii++) {
        knnCore->synapse_map[ii].discreteMapEntry = synMapEntry;
        synMapEntry.Ptr += synMapEntry.Len;
    }
    // setup input axons for refractory 1:1 input
    synMapEntry.Len = 1;
    for (int ii=0; ii<NEURONS_PER_CORE_KNN; ii++) {
        knnCore->synapse_map[NEURONS_PER_CHIP_ROUTING+ii].discreteMapEntry = synMapEntry;
        synMapEntry.Ptr += synMapEntry.Len;
    }

    // setup synapse mem for refractory 1:1 input  
    int weight = -128;
    for (int ii=0; ii<NEURONS_PER_CORE_KNN; ii++) {
        knnCore->synapse_mem[SYN_MEM_LENGTH*NEURONS_PER_CHIP_ROUTING + ii] = (weight<<20) | (ii<<10) | (1<<4) | 2; //2 for 2nd synapse format
    }
    
    // setup output axons
    int axonPtr = 0;
    int axonLen = 0;
    for (int ii=0; ii<NEURONS_PER_CORE_KNN; ii++) {
        uint16_t neuronId = knnCoreNum*NEURONS_PER_CORE_KNN + ii; //which neuron is this on the chip?
        uint16_t targetLmt = (neuronId / COUNTERS_PER_LAKEMONT);
        uint16_t targetCounter = neuronId % COUNTERS_PER_LAKEMONT;
        //connect to individual spike counter
        knnCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_cpu_coreid(targetLmt), targetCounter+32);        
        axonLen++;
        //connect to shared spike counter
        knnCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_cpu_coreid(targetLmt), COUNTERS_PER_LAKEMONT+(targetCounter/NEURONS_PER_SUM_COUNTER) +32); 
        axonLen++;
        //connect to self (refractory inhibition)
        knnCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_coreid(corenum), NEURONS_PER_CHIP_ROUTING + ii);        
        axonLen++;
        
        knnCore->axon_map[ii] = (AxonMapEntry){.Ptr = axonPtr, .Len = axonLen, .Atom = 0};
        axonPtr += axonLen;
        axonLen = 0;
    }
    nx_flush_core(nx_nth_coreid(corenum));
}


//corenum specifies which core to configure
//routingCoreNum specifies which of the routing cores this is
void setupRoutingCore(int corenum, int routingCoreNum) {
    
    bool chip_below = is_there_a_chip_below(my_chip_id); //if there is a chip below, always receive results from below
    bool chip_above = is_there_a_chip_above(my_chip_id); //if there is a chip above, always send results up
    bool chip_right = is_there_a_chip_right(my_chip_id); //if chip on top rung, forward to your right if such a chip exists
    
    // setup general core parameters
    NeuronCore* routingCore = NEURON_PTR(nx_nth_coreid(corenum));
    
    routingCore->synapse_fmt[1].WgtExp = 7;
    routingCore->synapse_fmt[1].WgtBits = 7;
    routingCore->synapse_fmt[1].NumSynapses = 63;
    routingCore->synapse_fmt[1].IdxBits = 5;
    routingCore->synapse_fmt[1].FanoutType = 1;
    
    routingCore->dendrite_accum_cfg.DelayBits = 0;
    
    routingCore->dendrite_shared_cfg.PosVmLimit = 7;
    routingCore->dendrite_shared_cfg.NegVmLimit = 23;
    routingCore->dendrite_shared_cfg.DmOffsets = 1;
    routingCore->dendrite_shared_cfg.DsOffset = 1;
    
    routingCore->cx_profile_cfg[0].RefractDelay = 1;
    routingCore->cx_profile_cfg[0].Decay_u = (1<<12)-1;
    routingCore->cx_profile_cfg[0].Decay_v = (1<<12)-1;
    
    routingCore->vth_profile_cfg[0].vthProfileStaticCfg.Vth = 1;
    routingCore->num_updates.num_updates = (NEURONS_PER_CORE_ROUTING+3)/4;
    
    // setup input axons (synapse map) for 1:1 routing
    DiscreteMapEntry synMapEntry;
    synMapEntry.CxBase = 0;
    synMapEntry.Len = 1;
    synMapEntry.Ptr = 0;
    for (int ii=0; ii<NEURONS_PER_CORE_ROUTING; ii++) {
        synMapEntry.Ptr = ii;
        routingCore->synapse_map[ii].discreteMapEntry = synMapEntry;
    }
    
    // setup synapse mem for 1:1 routing
    int32_t weight = 1;
    for (int ii=0; ii<NEURONS_PER_CORE_ROUTING; ii++) {
        routingCore->synapse_mem[ii] = (weight<<20) | (ii<<10) | (1<<4) | 1;
    }
    
    // setup output axons to connect to KNN
    int32_t axonPtr = 0;
    int32_t axonLen = 0;
    for (int ii=0; ii<NEURONS_PER_CORE_ROUTING; ii++) {
        //connect to the KNN cores
        for (int jj=0; jj<NUM_KNN_CORES; jj++) {
            int targetCore = NEURON_START_CORE_KNN + jj; //target cores start where KNN cores start
            int targetAxon = NEURONS_PER_CORE_ROUTING*routingCoreNum + ii; //leave space for other routing cores before these axons
            routingCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_coreid(targetCore), targetAxon);
            axonLen++;
        }
        
        // Input Routing to Adjacent Chips
        
        if (chip_above) {
            routingCore->axon_cfg[axonPtr+axonLen].remote = AXON_CFG_REMOTE(get_chipid_above(nx_my_chipid()));
            axonLen++;
            routingCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_coreid(corenum), ii);
            axonLen++;
        }
        if ((!chip_below) && chip_right) {
            // If there is no chip below the current chip, the forward to right chip
            routingCore->axon_cfg[axonPtr+axonLen].remote = AXON_CFG_REMOTE(get_chipid_right(nx_my_chipid()));
            axonLen++;
            routingCore->axon_cfg[axonPtr+axonLen].discrete = AXON_CFG_DISCRETE(nx_nth_coreid(corenum), ii);
            axonLen++;
        }
        
        routingCore->axon_map[ii] = (AxonMapEntry){.Ptr = axonPtr, .Len = axonLen, .Atom = 0};
        axonPtr += axonLen;
        axonLen = 0;
    }
    nx_flush_core(nx_nth_coreid(corenum));
}

void setupCoreMap() {
    // Prepopulate the core lookup table (done only once at the first timestep)
    for(int ii=0; ii<128; ii++)
        core_map[ii] = nx_nth_coreid(ii);
}

void setupTimeOffset() {
    // Fill this in with time offset for each lmt
    timeOffset = (isCpu0)? 2 : 1;
}

void setupChipScalabilityLimits() {
    // Setup max, min chip id and my chip id (the chip id on which this init snip is running)
    max_chip_id = nx_max_chipid();
    min_chip_id = nx_min_chipid();
    my_chip_id = nx_my_chipid();
}

void configureChipDelay() {
    ChipId firstChip = nx_nth_chipid(0);
    int diffX = my_chip_id.x - firstChip.x;
    int myY = CHIP_Y(my_chip_id) - CHIP_Y(firstChip);
    int diffY = myY; // MUST BE UPDATED FOR MULTI-HOP
    manhattanHopDelay = diffX + diffY; 
    timeStart = 1;
}

void reconfigure(runState *s) {
    //printf("Number of chips is %d\n",nx_num_chips());
    setupChipScalabilityLimits();
    logical_cpu_id = get_logical_cpu_id();
    logical_chip_id = get_logical_chip_id();
    
    isCpu0 = logical_cpu_id == 0;
    isCpu1 = logical_cpu_id == 1;
    isCpu2 = logical_cpu_id == 2;
    
    isFirstChip = my_chip_id.id == min_chip_id.id;
    isLastChip = my_chip_id.id == max_chip_id.id;
    
    isFirstCpu = isCpu0 && isFirstChip;
    isLastCpu = isCpu0 && isLastChip;
    isTimingCpu = isCpu0 && (logical_chip_id==1);
    
    if (isCpu1) {
        for (int ii=0; ii<NUM_KNN_CORES; ii++) {
            setupKnnCore(NEURON_START_CORE_KNN+ii, ii);
        }

        for (int ii=0; ii<NUM_ROUTING_CORES; ii++) {
            setupRoutingCore(NEURON_START_CORE_ROUTING+ii, ii);
        }
    }
        
    setupCoreMap();
    setupChannels();
    
    configureChipDelay();
    setupTimeOffset();
    
    msgIndex = (isLastCpu)? 2 : 0;
    tokensPerMessage = (isCpu0)? TOKENS_PER_REMOTE_MESSAGE : TOKENS_PER_LOCAL_MESSAGE;
    
    spikeTime = 63;
    spikeTarget = 1022;
}
