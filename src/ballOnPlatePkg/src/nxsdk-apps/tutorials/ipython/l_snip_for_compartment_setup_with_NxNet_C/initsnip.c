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

// The SNIP that runs in init phase to initialize cx0 parameters
#include "nxsdk.h"
#include "initsnip.h"

static int channelID = -1;

void initParams(runState *s) {
  if(channelID == -1) {
    channelID = getChannelID("nxinit");
    if(channelID == -1) {
      printf("Invalid channelID for nxinit\n");
    }
  }
// "nxCompartment" is a reserved key word. nxCompartment[0] refers to first compartment created in Python at NxNet level.
// In Python code, we have: cxGrp = net.createCompartmentGroup(size=2, prototype=[cxp0, cxp1], prototypeMap=[0, 1]),
// so compartment 0 (cx0) uses cxp0 prototype, and is initialized here.
  nxCompartment[0].Bias = 100;
  nxCompartment[0].BiasExp = 6;
  nxCompartment[0].Decay_v = 15;
  nxCompartment[0].RefractDelay = 2;
  nxCompartment[0].Vth = 1000;
}

