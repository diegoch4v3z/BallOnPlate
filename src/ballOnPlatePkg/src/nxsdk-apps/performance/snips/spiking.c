#include <stdlib.h>
#include "nxsdk.h"
#include "spiking.h"

static int time = 0;
static int chip;
static int core;
static int axon;
static int channelID = -1;

int doSpiking(runState *s) {

  // either done sending spikes (-1) or haven't reached the next time
  // step to inject spike
  if (time == -1 || time > s->time_step)
    return 0;

  // either haven't read any spike data yet or just sent a spike last
  // time step
  if (time < s->time_step) {
    if (channelID == -1) {
      channelID = getChannelID("nxspk");
      if (channelID == -1) {
        // could not find channel named nxspk
        return 0;
      }
    }
    
    readChannel(channelID, &time, 1);
  }

  // inject a spike at this time step
  if (time == s->time_step)
    return 1;
  else
    return 0;
}

void runSpiking(runState *s) {

  do {
    readChannel(channelID, &chip, 1);
    readChannel(channelID, &core, 1);
    readChannel(channelID, &axon, 1);

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

    readChannel(channelID, &time, 1);
    
  } while (time == s->time_step);
}
