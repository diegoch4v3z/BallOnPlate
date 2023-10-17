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

#ifndef KNN_SPIKING_H_
#define KNN_SPIKING_H_

#include "nxsdk.h"
#include "static_variables.h"



int doSpiking(runState *s);
void spiking(runState *s);
void stopOtherLakemonts();
void sendMsg(SpikeMessage* msg);
uint8_t analyzeMsg(SpikeMessage* msgIn, int msgSize);
void ack_and_drain_all_messages(int numPorts, CspRecvPort* ports);
#endif