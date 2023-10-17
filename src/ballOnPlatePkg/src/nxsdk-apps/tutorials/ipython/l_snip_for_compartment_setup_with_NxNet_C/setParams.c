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

// The SNIP that runs in mgmt phase to change compartments parameters
#include <stdlib.h>
#include <string.h>
#include "setParams.h"

bool setVoltageDecay = false;
bool setRefractoryDelay = false;
bool setVThreshold = false;

// Set up the guard fot settingSnip
int doSetting(runState *s) {
    if(s->time_step == 60){
        setVoltageDecay = true;
        return 1;
    }

    if(s->time_step == 150){
        setVThreshold = true;
        return 1;
    }

    if(s->time_step == 180){
        setRefractoryDelay = true;
        return 1;
    }

    return 0;
}

void setParams(runState *s) {
    // Get channels
    int settingChannelID = getChannelID("settingChannel");
    if( settingChannelID == -1 ) {
      printf("Invalid Channel ID\n");
      return;
    }

    // get the value used to set refractoryDelay
    static int refractoryDelay;
    if(s->time_step == 60){
        readChannel(settingChannelID,&refractoryDelay,1);
    }

    // at time step 60 (when setVoltageDecay is true) set Decay_v for the whole cxGrp
    // "nxCompartmentGroup" is a reserved key word. 
    // nxCompartmentGroup[0] refer to the first compartment group created in Python at NxNet level.  
    if(setVoltageDecay == true){
        nxCompartmentGroup[0].Decay_v = 256;
    }

    // at time step 150 (when setVThreshold is true) set Vth for the whole cxGrp
    if(setVThreshold == true){
        nxCompartmentGroup[0].Vth = 500;
    }

    // at time step 180 (when setRefractoryDelay is true) set RefractDelay for cx1  
    if(setRefractoryDelay == true){
        // either nxCompartmentGroup[0][1] can be used to refer cx1, or,
        // nxCompartment[1] can be used to refer cx1
        nxCompartmentGroup[0][1].RefractDelay= refractoryDelay;
        //nxCompartment[1].RefractDelay= refractoryDelay;
    }

    printf("Set cxGrp Decay=%d, ", nxCompartment[0].Decay_v);
    printf("vTh=%d, ", nxCompartment[0].Vth);
    printf("refractDelay=%d at time %d\n", nxCompartment[1].RefractDelay, s->time_step);
}
