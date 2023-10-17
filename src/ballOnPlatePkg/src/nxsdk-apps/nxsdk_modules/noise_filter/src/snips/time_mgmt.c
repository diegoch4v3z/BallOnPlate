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

#include "time_mgmt.h"

int do_time_mgmt(runState *s){
    return 1;
}

void time_mgmt(runState *s){
    //printf("Running time mgmt at timestep : %d\n",s->time_step);

    // The time at which we should advance to the next timestep
    global_target_time_us += global_time_step_duration_us;
    
    // wait until the time is reached
    while(timestamp() < global_target_time_us*TICKS_PER_MICROSECOND);

}