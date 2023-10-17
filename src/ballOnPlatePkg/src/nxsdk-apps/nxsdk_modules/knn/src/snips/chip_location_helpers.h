/*
INTEL CONFIDENTIAL

Copyright © 2020-2021 Intel Corporation.

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

#ifndef CHIP_LOCATION_HELPERS_H_
#define CHIP_LOCATION_HELPERS_H_

#include "nxsdk.h"
#include "static_variables.h"

#define CHIP_Y(chipid) ((chipid).id & 0x7f)

static int get_logical_chip_id() {
    int chipid = -1;
    for (int ii=0; ii<nx_num_chips(); ii++)
        if (nx_nth_chipid(ii).id == nx_my_chipid().id) {
            chipid = ii;
            break;
    }
    return chipid;
}

static int get_logical_cpu_id() {
    int cpuid = -1;
    for (int ii=0; ii<3; ii++)
        if (nx_my_coreid().id == nx_coreid_lmt(ii).id) {
            cpuid = ii;
            break;
        }
    return cpuid;
}

static int get_logical_core_id(int coreid) {
    CoreId thisCore;
    thisCore.id = coreid;
    int return_coreid = -1;

    for (int ii=0; ii<128; ii++)
        if (thisCore.id == nx_nth_coreid(ii).id) {
            return_coreid = ii;
            break;
        }
    return return_coreid;
}

static bool is_there_a_chip_right(ChipId my_id) {
  if (my_id.x < max_chip_id.x) {
    return 1;
  }
  return 0;
}

static bool is_there_a_chip_above(ChipId my_id) {
  if (CHIP_Y(my_id) < CHIP_Y(max_chip_id)) {
    return 1;
  }
  return 0;
}

static bool is_there_a_chip_below(ChipId my_id) {
  if (CHIP_Y(my_id) > CHIP_Y(min_chip_id)) {
    return 1;
  }
  return 0;
}

static bool is_there_a_chip_left(ChipId my_id) {
  if (my_id.x > min_chip_id.x) {
    return 1;
  }
  return 0;
}

static ChipId get_chipid_left(ChipId my_id) {
   my_id.x -= 1;
   return my_id;
}

static ChipId get_chipid_right(ChipId my_id) {
   my_id.x += 1;
   return my_id;    
}

static ChipId get_chipid_above(ChipId my_id) {
   uint16_t y = CHIP_Y(my_id);
   y += 1;
   my_id.z = y & 7;
   my_id.y = y >> 3;
   return my_id;
}

static ChipId get_chipid_below(ChipId my_id) {
   uint16_t y = CHIP_Y(my_id);
   y -= 1;
   my_id.z = y & 7;
   my_id.y = y >> 3;
   return my_id;
}

#endif
