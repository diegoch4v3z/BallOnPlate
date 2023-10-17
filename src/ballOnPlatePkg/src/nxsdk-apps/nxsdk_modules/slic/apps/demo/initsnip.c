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

// Number fo compartments in the network
int numCx;
// Number of axons in the network
int numAxon;
// CoreId where the network is located
int coreId;
// Number of timesteps for each image
int timestepPerImage;
// Number of training iterations
int numTrainIterations;
// timestep at which first image is present
int imagePresentOffset;
// Size of training dataset
int numTrainImages;
// Size of each image
int bytesPerImage;
// First timestep
int firstTimestep;
// Interval at which spikes will be inserted for an image
int spikeInterval;

/* Helper function to print network parameters */
void print_network_parameters(){
    printf("numCx : %d\n",numCx);
    printf("numAxon : %d\n",numAxon);
    printf("coreId : %d\n",coreId);
    printf("timestepPerImage : %d\n",timestepPerImage);
    printf("numTrainIterations : %d\n",numTrainIterations);
    printf("imagePresentOffset : %d\n",imagePresentOffset);
    printf("numTrainImages : %d\n",numTrainImages);
    printf("bytesPerImage : %d\n",bytesPerImage);
    printf("firstTimestep : %d\n",firstTimestep);
    printf("spikeInterval : %d\n",spikeInterval);
}

/* Function to load the model parameters from the channel */
void initsnip(runState *s) {
    int channelID = getChannelID("nxinit");
    readChannel(channelID, &numCx, 1);
    readChannel(channelID, &numAxon, 1);
    readChannel(channelID, &coreId, 1);
    readChannel(channelID, &timestepPerImage, 1);
    readChannel(channelID, &numTrainIterations, 1);
    readChannel(channelID, &imagePresentOffset, 1);
    readChannel(channelID, &numTrainImages, 1);
    readChannel(channelID, &bytesPerImage, 1);
    readChannel(channelID, &firstTimestep, 1);
    readChannel(channelID, &spikeInterval, 1);
}
