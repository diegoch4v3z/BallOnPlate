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

#include "postlearnmgmt.h"

//this function executes on every time step. We need to return 1 only on timesteps where learning occurs
//can this be done more efficiently (built in function?) than manually matching tepoch?
int do_postlearn_mgmt(runState *s){
    //if we are in a tEpoch
    if(s->time_step%s->epoch == 0){
        return 1;
    }
    return 0;
}

void postlearn_mgmt(runState *s){
    //printf("Running post-learn mgmt at Time : %d\n",s->time_step);
    //int time;
    CoreId core;
    NeuronCore *nc;
    
    //update the CIdxOffset
    for(int ii=0; ii<num_rewiring_cores; ii++)
    {
        core.id = global_coreId[ii];
        nc = NEURON_PTR(core);

        nc->synapse_fmt[global_synFmtIndex[ii]].CIdxOffset = 0;
    }
     // Read Something to flush the write
    //time = nc->time.Time;
    //printf("Post-learn management completed at time t=%d\n", time);
}