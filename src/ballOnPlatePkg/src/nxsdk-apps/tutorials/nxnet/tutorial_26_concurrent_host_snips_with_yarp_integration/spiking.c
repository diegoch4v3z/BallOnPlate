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

#include "spiking.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NUM_CX 4
#define RUN_FOR_TIME_STEPS 14

  // Channel Id of the feedback channel
  int feedbackChannelId = -1;
  // Channel Id of the input channel
  int inputChannelId = -1;

int doSpiking(runState *s) {
  if (s->time_step == 1) {
    feedbackChannelId = getChannelID("feedback");
    inputChannelId = getChannelID("input");
  }
  return 1;
}

void runSpiking(runState *s) {
  // Array to hold the addresses of cx spiked
  int cntrIdx[NUM_CX] = {0};
  // Stores the timestamp in which cx spiked
  int timeStamp = 0;
  // Num of cx spiked in the time step
  int numSpikes = 0;
  // Id of the cx in which spike needs to be inserted
  int cxId = -1;
  // Flag variable to indicate if its last time step
  int sentinel = 0;

  // Reading the cxId from the input channel
  readChannel(inputChannelId, &cxId, 1);

  uint16_t axon = cxId;
  uint16_t axonId = 1 << 14 | (axon & 0x3FFF);
  // Sending spike to provided address
  nx_send_discrete_spike(s->time_step, nx_nth_coreid(0), axonId);

  // Resetting numSpikes to 0
  numSpikes = 0;

  // Reading the spike counters to check which cx spiked
  // Incrementing numSPikes and storing the cxId in cntrIdx array
  for (int i = 0; i < NUM_CX; i++) {
    if (SPIKE_COUNT[(s->time_step - 1) & 3][0x20 + i]) {
      cntrIdx[numSpikes] = i;
      numSpikes++;
    }
    SPIKE_COUNT[(s->time_step - 1) & 3][0x20 + i] = 0;
  }

  // Writing the numSpikes to indicate how much more message is to be read
  writeChannel(feedbackChannelId, &numSpikes, 1);

  if (numSpikes) {
    timeStamp = s->time_step;
    // Writing the timestamp on feedback channel
    writeChannel(feedbackChannelId, &timeStamp, 1);
    // Writing all the spiked cxId
    writeChannel(feedbackChannelId, cntrIdx, numSpikes);
  }

  // Checking if the sentinel needs to be changed
  // sentinel is being used to tell concurrent host snip to quit
  if (s->time_step == RUN_FOR_TIME_STEPS) {
    sentinel = -1;
  }
  // Writing the value of sentinel
  writeChannel(feedbackChannelId, &sentinel, 1);
}
