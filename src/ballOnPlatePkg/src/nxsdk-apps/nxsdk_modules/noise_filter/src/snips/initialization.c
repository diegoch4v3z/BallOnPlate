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

#include "initialization.h"

int global_time_step_duration_us;
uint64_t global_target_time_us = 0;

void initParams(runState *s) {
    int channelID;
        
    channelID = getChannelID("nxTimeInit");
    if(channelID == -1) {
        printf("Invalid channelID for nxTimeInit\n");
    }
    
    readChannel(channelID,&global_time_step_duration_us,1);    
}
