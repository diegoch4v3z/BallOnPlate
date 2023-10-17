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
#include "runmgmt.h"

int imageLabels;
int labelChannelId;
static CoreId core;
static NeuronCore *neuron;

/* Function used to Reset the CxState and Traces*/
void ResetTraceandCxState()
{
    nx_fast_init(&neuron->synapse_map[1], numAxon, 4, 8, (uint32_t[1]) {0});
    nx_fast_init32(&neuron->cx_meta_state, numCx, 0);
    nx_fast_init32(&neuron->cx_state, 2 * numCx, 0);
}

/* Function to set supervisory learning labels and switch the
   learning rules*/
void SetLabel(int label)
{
    for (int i = 0; i < numCx; i++) {
        neuron->stdp_post_state[i] = (PostTraceEntry) {
            .Yspike0      = 0,
            .Yspike1      = 0,
            .Yspike2      = 0,
            .Yepoch0      = 0,
            .Yepoch1      = 0,
            .Yepoch2      = 0,
            .Tspike       = 0,
            .TraceProfile = 3,
            .StdpProfile  = i == label
        };
    }
}

/* Function to disable the learning. Used before inference.*/
void DisableLearning()
{
    neuron->num_updates = (UpdateCfg) {
        .num_updates = numCx / 4 + 1,
        .num_stdp    = 0
    };
}

/* Function to clear the values of activity after processing each image during inference */
void ClearSomaState()
{
    nx_fast_init32(&neuron->soma_state, 2 * numCx, 0);
}

/* Function to decide when to do management */
int do_run_mgmt(runState *RunState) {
    // Load the address of the neuroCore in which the network is located on first timestep
    if(RunState->time_step == 1){
        core.id = coreId;
        neuron = NEURON_PTR(core);
        labelChannelId = getChannelID("nxlabel");
    }
    // Check if its time to do management
    if(RunState->time_step%timestepPerImage == imagePresentOffset) {
        return 1;
    } else {
        return 0;
    }
}

/* Function to do run management */
void run_mgmt(runState *RunState) {
    // Reset the state of comaprtments and traces
    ResetTraceandCxState();
    // If training phase
    if(trainingPhase == 1) {
        if(RunState->time_step >= timestepPerImage*numTrainIterations*numTrainImages + firstTimestep) {
            trainingPhase = 0;
            ClearSomaState();
            DisableLearning();
        }
        // If training is not complete :
        // a) Read the label of the next image to be trained
        // b) Set the learning rules corresponding to it
        else {
            readChannel(labelChannelId, &imageLabels, 1);
            SetLabel(imageLabels);
        }
    }
    nx_flush_core(core);
}
