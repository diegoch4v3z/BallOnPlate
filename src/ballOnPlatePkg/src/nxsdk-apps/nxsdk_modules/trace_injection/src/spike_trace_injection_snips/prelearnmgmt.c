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

#include "prelearnmgmt.h"

int do_prelearn_mgmt(runState *s){
    return 1;
}

void prelearn_mgmt(runState *s){
    //printf("Running pre-learn mgmt at Time : %d\n",s->time_step);
    //int time;

    CoreId core;
    NeuronCore *nc;
    
    //-------------- core rewiring    
    for (int ii=0; ii<num_rewiring_cores; ii++)
    {
        core.id = global_coreId[ii];
        nc = NEURON_PTR(core);

        //get the offset value we will use on this iteration
        int compartmentIdxOffset=global_connection_order[ii][global_connection_index[ii]];

        //update the CIdxOffset
        nc->synapse_fmt[global_synFmtIndex[ii]].CIdxOffset = compartmentIdxOffset;

        //increment the connection_index and wrap around if necessary 
        global_connection_index[ii]=global_connection_index[ii]+1;
        if (global_connection_index[ii]>=global_connection_order_length[ii])
            global_connection_index[ii]=0;
    }
    
    //time = nc->time.Time;
    //printf("Pre-learn management completed at time t=%d\n", time);
}