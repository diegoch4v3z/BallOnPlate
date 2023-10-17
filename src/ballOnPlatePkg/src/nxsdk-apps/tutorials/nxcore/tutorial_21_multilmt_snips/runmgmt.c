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

int do_run_mgmt(runState *s) {
    if(s->time_step%3==0)
        return 1;
    else
        return 0;
}

void run_mgmt(runState *s) {
    printf("Runnning Mgmt at Time : %d\n",s->time_step);
    CoreId core;
    core.id = 4;
    int numCx = 2;
    NeuronCore *nc = NEURON_PTR(core);
    // reset cx state every third cycle
    nx_fast_init32(&nc->cx_state, 2 * numCx, 0);
}
