/*
INTEL CONFIDENTIAL

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
#include "reset_query.h"
#include <unistd.h>
#include "static_variables.h"

int do_run_mgmt(runState *s) {
    // This will be run on the CPU0 of last chip (controlled via python createSnip API)
    return kSolutionsFound || (s->time_step==1); 
}


void run_mgmt(runState *s) {
    return;
}
