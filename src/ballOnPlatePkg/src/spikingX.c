/* 
Diego Chavez 
New Mexico State University 
Fall 2023
*/
#include "spiking.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "nxsdk.h"

#define NUM_CONNECTIONS 100 
#define RUN_TIME 300

int feedbackChannelId = -1; //Store the IDs of the communication channels
int inputChannelId = -1; //Store the IDs for the communication channels
int sentinel = 0; //Sentinel value to check if the simulation is done

int dSpiking(runState *s) {
  if (s->time_step == 1) {
    feedbackChannelId = getChannelID("feedback"); //Get ID for the feedback channel
    inputChannelId = getChannelID("input"); //Get ID for the input channel
  }
  return 1;
}

void rSpiking(runState *s) {
    int coord[2];
    int cntrIDx[NUM_CONNECTIONS] = {0}; //Array to store the indices of the neurons that spiked
    int xId = -1; 
    int numSpikes = 0;

    readChannel(inputChannelId, &coord, 2); //Read the coordinates from the input channel
    xId = coord[0]; 


    //Injection of spikes
    // Unsigned integer of 16 bits. It can represent 2^16 or 0 to 65,535
    uint16_t axonX = xId;
    // Most likely there is a limited number of axons where you can inject spikes
    axonX = 1 << 14 | (axonX & 0x3FFF); 
    nx_send_discrete_spike(s->time_step, nx_nth_coreid(0), axonX); // Send spikes to a specific core 

  //Lakemont spike counter is located at 0x21
  //Maybe each recording of spike happens in three timesteps
    for(int i = 0; i < NUM_CONNECTIONS; i++){
        if(SPIKE_COUNT[(s->time_step - 1) & 3][0x20+i]) {
            cntrIDx[numSpikes] = i; 
            numSpikes++;
        }
        SPIKE_COUNT[(s->time_step - 1) & 3][0x20 + i] = 0; //reset the counter value

    }

    writeChannel(feedbackChannelId, &numSpikes, 1); // Indicate how many spikes is needed to be read 

    if (numSpikes) {
      int timestamp = s->time_step;
      writeChannel(feedbackChannelId, &timestamp, 1);
      //might be better to store the 
      writeChannel(feedbackChannelId, cntrIDx, numSpikes); 
    }

    if (s->time_step == RUN_TIME) {
      sentinel = -1; 
    }

    writeChannel(feedbackChannelId, &sentinel, 1); // send the sentinel value to check 
    

}