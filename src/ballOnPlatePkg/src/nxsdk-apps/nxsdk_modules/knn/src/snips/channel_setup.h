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

#ifndef CHANNEL_SETUP_H_
#define CHANNEL_SETUP_H_

#include "chip_location_helpers.h"
#include "static_variables.h"
#include "params.h"

static void setupChannels() {    
    
    if (isCpu0) {
        csp_create_recv_port(nx_my_chipid(), nx_coreid_lmt(1), &(localReceivePortArray[numLocalReceivePorts++]), LOCAL_MESSAGE_BYTE_SIZE, NUM_MESSAGES);
        csp_create_recv_port(nx_my_chipid(), nx_coreid_lmt(2), &(localReceivePortArray[numLocalReceivePorts++]), LOCAL_MESSAGE_BYTE_SIZE, NUM_MESSAGES);
    } else {
        csp_create_send_port(nx_my_chipid(), nx_coreid_lmt(0), &sendPort);
    }
    
    if (isCpu0) {
        if (is_there_a_chip_above(my_chip_id)) {
            csp_create_send_port(get_chipid_above(my_chip_id), nx_coreid_lmt(0), &sendPort);
        } else if (is_there_a_chip_right(my_chip_id)) {
            csp_create_send_port(get_chipid_right(my_chip_id), nx_coreid_lmt(0), &sendPort);
        } 

        if (is_there_a_chip_left(my_chip_id) && !is_there_a_chip_above(my_chip_id)) {
            csp_create_recv_port(get_chipid_left(my_chip_id), nx_coreid_lmt(0), &remoteReceivePortArray[numRemoteReceivePorts++], REMOTE_MESSAGE_BYTE_SIZE, NUM_MESSAGES); 
        }
        if (is_there_a_chip_below(my_chip_id)) {
           csp_create_recv_port(get_chipid_below(my_chip_id), nx_coreid_lmt(0), &remoteReceivePortArray[numRemoteReceivePorts++], REMOTE_MESSAGE_BYTE_SIZE, NUM_MESSAGES); 
        }
    }
    
    if(isFirstCpu) {
        spikeChannelID = getChannelID("spikeAddresses");
        if (spikeChannelID == -1) {
            printf("Invalid channelID for spikeAddresses\n");
        }
    }
    
    if (isLastCpu) {
        resultChannelID = getChannelID("results");
        if (resultChannelID == -1) {
            printf("Invalid channelID for channel named results\n");
        }
    }
    
    #ifdef BENCHMARK
    if (isTimingCpu) {
        timingChannelID = getChannelID("timingInfo");
        if( timingChannelID == -1) {
          printf("Invalid Timing Channel ID\n");
          return;
        }
    }
    #endif
}

#endif
