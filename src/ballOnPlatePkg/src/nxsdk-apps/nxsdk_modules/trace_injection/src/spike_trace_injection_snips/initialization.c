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

// for synapse rewiring
int global_synFmtIndex[num_rewiring_cores];
int global_coreId[num_rewiring_cores];
int global_connection_order[num_rewiring_cores][max_connection_order_length];
int global_connection_order_length[num_rewiring_cores];
int global_connection_index[num_rewiring_cores];


void initParams(runState *s) {
    int channelID;
        
    //------------------initialize core rewiring
    if(num_rewiring_cores>0)
    {
        channelID = getChannelID("nxSynFmtinit");
        if(channelID == -1) {
          printf("Invalid channelID for nxSynFmtinit\n");
        }
        readChannel(channelID,global_synFmtIndex,num_rewiring_cores); //get the synapse_fmt_register index (used by other functions)

        channelID = getChannelID("nxCoreIdinit");
        if(channelID == -1) {
          printf("Invalid channelID for nxCoreIdinit\n");
        }
        readChannel(channelID,global_coreId,num_rewiring_cores); //get the synapse_fmt_register index (used by other functions)

        channelID = getChannelID("nxConnectionOrderLengthInit");
        if(channelID == -1) {
          printf("Invalid channelID for nxConnectionOrderLengthInit\n");
        }
        readChannel(channelID,global_connection_order_length,num_rewiring_cores); //get the length of the list for each core

        channelID = getChannelID("nxConnectionOrderInit");
        if(channelID == -1) {
          printf("Invalid channelID for nxConnectionOrderInit\n");
        }
        for(int ii=0; ii<num_rewiring_cores; ii++)
            readChannel(channelID,global_connection_order[ii],global_connection_order_length[ii]); //get the rewiring order
    }
    
}
