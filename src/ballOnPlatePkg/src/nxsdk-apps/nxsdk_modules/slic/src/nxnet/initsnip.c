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

// Number of compartments in the network
int numCx;
// Number of iterations for training images
int numTrainIterations;
// First timestep at which image is present
int imagePresentOffset;
// Sized of training dataset
int numTrainImages;

/* Helper function to print network parameters */
void print_network_parameters(){
    printf("numCx : %d\n",numCx);
    printf("numTrainIterations : %d\n",numTrainIterations);
    printf("imagePresentOffset : %d\n",imagePresentOffset);
    printf("numTrainImages : %d\n",numTrainImages);
}

/* Function to load the model parameters from the channel */
void initsnip(runState *s) {
    int channelID = getChannelID("nxinit");
    readChannel(channelID, &numCx, 1);
    readChannel(channelID, &numTrainIterations, 1);
    readChannel(channelID, &imagePresentOffset, 1);
    readChannel(channelID, &numTrainImages, 1);
}
