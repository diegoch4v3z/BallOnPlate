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
    
    if (global_overwrite_y1)
    {
        int y1_trace[num_overwrite_compartments];
        
        int channelID = getChannelID("nxY1Trace");
        if(channelID == -1)
            printf("Invalid channelID for nxY1Trace\n");
        
        readChannel(channelID,y1_trace,num_overwrite_compartments); //get the trace values
        
        for(int ii=0; ii<num_overwrite_compartments; ii++)
            global_post_trace_entry[ii].Yepoch0=y1_trace[ii];
    }
    
    if (global_overwrite_y2)
    {
        int y2_trace[num_overwrite_compartments];

        int channelID = getChannelID("nxY2Trace");
        if(channelID == -1)
            printf("Invalid channelID for nxY2Trace\n");
        
        readChannel(channelID,y2_trace,num_overwrite_compartments); //get the trace values
        
        for(int ii=0; ii<num_overwrite_compartments; ii++)
            global_post_trace_entry[ii].Yepoch1=y2_trace[ii];
    }
    
    if (global_overwrite_y3)
    {
        int y3_trace[num_overwrite_compartments];

        int channelID = getChannelID("nxY3Trace");
        if(channelID == -1)
            printf("Invalid channelID for nxY3Trace\n");
        
        readChannel(channelID,y3_trace,num_overwrite_compartments); //get the trace values
        for(int ii=0; ii<num_overwrite_compartments; ii++)
            global_post_trace_entry[ii].Yepoch2=y3_trace[ii];
    }
        
    
    CoreId core;
    NeuronCore *nc;    
    
    //write to the stdp_post_state registers
    for(int ii=0; ii<num_overwrite_compartments; ii++)
    {
        core.id = global_overwrite_core_ids[ii];
        nc = NEURON_PTR(core);

        nc->stdp_post_state[global_overwrite_compartment_ids[ii]] = global_post_trace_entry[ii];
        //printf("Item %d writing core %d STDP_POST_STATE register %d y1 trace with value %d\n", ii, global_overwrite_core_ids[ii], global_overwrite_compartment_ids[ii], global_post_trace_entry[ii].Yepoch0);
    }
    
    core.id = 4;
    nc = NEURON_PTR(core);
    //int time = nc->time.Time;
    //printf("Pre-learn management for completed at time t=%d\n", time);
}