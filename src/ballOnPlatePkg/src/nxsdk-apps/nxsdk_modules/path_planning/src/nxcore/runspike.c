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

#include <stdlib.h>
#include <string.h>
#include "runspike.h"
#include <time.h>
#include <unistd.h>

int didSpike = 0;
int channelID = -1;
int sourceSpikeTime = -1;

static int get_logical_chip(void) {
  for (int i = 0; i < nx_num_chips(); ++i) {
    if (nx_nth_chipid(i).id == nx_my_chipid().id) {
      return i;
    }
  }
  return -1;
}

int do_run_spike(runState *s) {
  if (s->time_step==1){
    int logical_chip = get_logical_chip();
    if (logical_chip == -1) {
      printf("Can't find logical chip ID %x\n", nx_my_chipid().id);
      exit(1);
    }

    char name[16];
    sprintf(name, "nxspktime_%d", logical_chip);
    channelID = getChannelID(name);
    if (channelID == -1) {
      printf("Can't find channel %s\n", name);
      exit(1);
    }
  }
  return 1;
}

void run_spike(runState *s) {
  int spikeTime = s->time_step-1;
  didSpike = SPIKE_COUNT[spikeTime&3][0x20]; // This macro is used to read the lakemont spike counter
  SPIKE_COUNT[spikeTime&3][0x20] = 0; // Lakemont spike counters need to be cleared after reading to prevent overflow
  ////writeChannel(channelID, &didSpike, 1);
  // Write the spike time value back to the channel
  if(didSpike) {
    //printf("Yay it spiked\n");
    writeChannel(channelID, &spikeTime, 1);
    sourceSpikeTime = spikeTime;
  }
  if(sourceSpikeTime>0 && (spikeTime)%s->epoch==0) {
    writeChannel(channelID, &spikeTime, 1);
    sourceSpikeTime = -1;
  }
}
