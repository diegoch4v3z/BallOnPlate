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

#include "initialization.h"

int global_overwrite_core_ids[num_overwrite_compartments];
int global_overwrite_compartment_ids[num_overwrite_compartments]; //stdp_post_state is indexed by compartmentId

int global_overwrite_y1;
int global_overwrite_y2;
int global_overwrite_y3;

PostTraceEntry global_post_trace_entry[num_overwrite_compartments];



void initParams(runState *s) {
    
    int channelID; //we'll reuse this for all initialization channels
    
    //which traces will we be modifying (Y1/Y2/Y3)?
    channelID = getChannelID("nxWhichY");
    if(channelID == -1) {
      printf("Invalid channelID for nxWhichY\n");
    }
    int whichy[3];
    readChannel(channelID,whichy,3);
        
    global_overwrite_y1 = whichy[0];
    global_overwrite_y2 = whichy[1];
    global_overwrite_y3 = whichy[2];
    
    
    //which coreIds do our compartments lie on?
    channelID = getChannelID("nxinitCoreIds");
    if(channelID == -1) {
      printf("Invalid channelID for nxinitCoreIds\n");
    }
    readChannel(channelID,global_overwrite_core_ids,num_overwrite_compartments);
    
    
    //which stdp state registers are we modifying
    channelID = getChannelID("nxinitStdpCompartmentIndex");
    if(channelID == -1) {
      printf("Invalid channelID for nxinitStdpCompartmentIndex\n");
    }
    readChannel(channelID,global_overwrite_compartment_ids,num_overwrite_compartments);
    
    
    //----------------what are the stdp post trace states states at initialization?
    // (since we cannot read them while the chip is running)
    
    // stdp profile
    int stdpProfile[num_overwrite_compartments];
    channelID = getChannelID("nxinitStdpProfile");
    if(channelID == -1) {
      printf("Invalid channelID for nxinitStdpProfile\n");
    }
    readChannel(channelID,stdpProfile,num_overwrite_compartments);
    
    // trace profile
    int traceProfile[num_overwrite_compartments];
    channelID = getChannelID("nxinitTraceProfile");
    if(channelID == -1) {
      printf("Invalid channelID for nxinitTraceProfile\n");
    }
    readChannel(channelID,traceProfile,num_overwrite_compartments);

        // explicitly initialize the values for all fields. Probably unnecessary, but I'm not sure what they do?
    // TraceProfile presumably determines trace behaviour (decay/impulse) but we'll overwrite it at every timestep anyway, so shouldn't matter
    // stdpProfile may be involved in determining which stdp rule to use, so we should make sure it is correct
    // Tspike probably indicates when a spike arrived? Doesn't matter since we overwrite traces
    // epoch or spike probably encode the trace
    for(int ii=0; ii<num_overwrite_compartments; ii++)
    {
        global_post_trace_entry[ii].Yspike0=0;
        global_post_trace_entry[ii].Yspike1=0;
        global_post_trace_entry[ii].Yspike2=0;
        global_post_trace_entry[ii].Yepoch0=0;
        global_post_trace_entry[ii].Yepoch1=0;
        global_post_trace_entry[ii].Yepoch2=0;
        global_post_trace_entry[ii].Tspike=0;
        global_post_trace_entry[ii].TraceProfile=traceProfile[ii];
        global_post_trace_entry[ii].StdpProfile=stdpProfile[ii];
    }

}
