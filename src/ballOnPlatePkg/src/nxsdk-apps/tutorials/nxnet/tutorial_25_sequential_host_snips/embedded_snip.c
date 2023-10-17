/*
Copyright © 2018-2021 Intel Corporation.

The source code contained or described herein and all documents
related to the source code ("Material") are owned by Intel Corporation
or its suppliers or licensors.  Title to the Material remains with
Intel Corporation or its suppliers and licensors.  The Material may
contain trade secrets and proprietary and confidential information of
Intel Corporation and its suppliers and licensors, and is protected by
worldwide copyright and trade secret laws and treaty provisions.  No
part of the Material may be used, copied, reproduced, modified,
published, uploaded, posted, transmitted, distributed, or disclosed in
any way without Intel's prior express written permission.  No license
under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or
delivery of the Materials, either expressly, by implication,
inducement, estoppel or otherwise. Any license under such intellectual
property rights must be express and approved by Intel in writing.
Unless otherwise agreed by Intel in writing, you may not remove or
alter this notice or any other notice embedded in Materials by Intel
or Intel's suppliers or licensors in any way.
*/

#include <stdlib.h>
#include "nxsdk.h"

// All axons in this tutorial are on chip 0 and core 4 which
// represent the first allocated core
static int chip = 0;
static int core = 4;
static int axon = 0;

// Channel for communication between host and embedded snip
static int channelID = -1;

// Enable do spiking on all timesteps
int doPhase(runState *s) {
    if (channelID == -1) {
      channelID = getChannelID("axon_info");
      if (channelID == -1) {
        // could not find channel named nxspk
        return 0;
      }
    }
    return 1;
}

void runPhase(runState *s) {
    // reads one integer from channel which represents the axon id
    readChannel(channelID, &axon, 1);

    // convert logical chip id and logical axon id to physical ids
    uint16_t  axonId = 1<<14 | (axon & 0x3FFF);
    ChipId    chipId = nx_nth_chipid(chip);

    int time = s->time_step;

    // send the spike
    nx_send_remote_event(time, chipId, (CoreId){.id=core}, axonId);
}
