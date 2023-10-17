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

#include "initsnip.h"
#include "nxsdk.h"

void initA() {
    int i =0;
    int data[15];
    int imageChannelID = getChannelID("imageChannel");
    int recvChannelID  = getChannelID("recvChannel");
    if( imageChannelID == -1 || recvChannelID == -1 ) {
      printf("Invalid channelID\n");
      return;
    }
    
    printf("From init function :\n");

    // Reading 15 elements from the imageChannel
    for(i=0;i<15;i++){
      readChannel(imageChannelID,data+i,1);
    }
    
    // Incrementing the data received by 4
    for(i=0;i<15;i++){
      data[i] = data[i]+4;
    }

    // Sending the incremented data to Superhost via the recvChannel
    for(i=0;i<15;i++){
      writeChannel(recvChannelID,data+i,1);
    }
}
