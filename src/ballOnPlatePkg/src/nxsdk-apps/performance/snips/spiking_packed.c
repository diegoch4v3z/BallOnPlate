#include <stdlib.h>
#include "nxsdk.h"
#include "spiking_packed.h"

static int time = 0;
static int chip;
static int core;
static int axon;
static int channelID = -1;
static int32_t spikeData[16] = {0};
static int numSpikeMsgs = 0;
static int index = 0;
static int processedSpikes = 0;
static int nextTimetoRead = 0;

int doSpiking(runState *s) {

  // either not time to read again or haven't reached the next time
  // step to inject spike
  if (nextTimetoRead > s->time_step || time > (int)s->time_step) {
    return 0;
  }

  if (channelID == -1) {
    channelID = getChannelID("nxspk");
    if (channelID == -1) {
      // could not find channel named nxspk
      return 0;
    }
  }
  
  if (time < (int)s->time_step) {
    readChannel(channelID, spikeData, 1);

    processedSpikes = 0;
    index = 0;
    numSpikeMsgs = spikeData[index++];
    time         = spikeData[index++];

    if (time < 0 ) {
      int numSteps   = -time;
      nextTimetoRead = s->time_step + numSteps - 1;
    }
  }

  // inject a spike at this time step
  if (time == s->time_step)
    return 1;
  else
    return 0;
}

void runSpiking(runState *s) {

  do {
    chip = spikeData[index++];
    core = spikeData[index++];
    axon = spikeData[index++];

    // convert logical chip id and logical axon id to physical ids
    uint16_t  axonId = 1<<14 | (axon & 0x3FFF);
    ChipId    chipId = nx_nth_chipid(chip);

    // TODO: does not handle long spikes
    // send the spike
    //if (axon >> 48) {
    //  nx_send_remote_long_event(time, chipId, (CoreId){.id=core}, axonId);
    //} else {
    //  nx_send_remote_event(time, chipId, (CoreId){.id=core}, axonId);
    //}
    nx_send_remote_event(time, chipId, (CoreId){.id=core}, axonId);

    processedSpikes++;

    if ((processedSpikes == numSpikeMsgs) && (numSpikeMsgs == 3)) {
      processedSpikes = 0;
      index = 0;

      readChannel(channelID, spikeData, 1);
      numSpikeMsgs = spikeData[index++];
    }

    if (spikeData[0] < 0 ) {
      time = spikeData[0];
    } else {
      time = spikeData[index++];
    }

    if( time < 0 ) {
      int numSteps   = -time;
      nextTimetoRead = s->time_step + numSteps;
    }

  } while (time == s->time_step);
}
