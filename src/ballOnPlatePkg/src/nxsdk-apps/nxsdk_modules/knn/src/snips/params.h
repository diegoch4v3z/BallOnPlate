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

#ifndef PARAMS_H_
#define PARAMS_H_

#define BENCHMARK

#define TOKENS_PER_PACKET 256
#define K 100
#define THRESHOLD_KNN 64000

#define NEURONS_PER_CHIP_KNN 2400
#define NEURONS_PER_CORE_KNN 32
#define NEURON_START_CORE_KNN 0

#define NEURONS_PER_CHIP_ROUTING 1024
#define NEURONS_PER_CORE_ROUTING 32
#define NEURON_START_CORE_ROUTING ((NEURONS_PER_CHIP_KNN + (NEURONS_PER_CORE_KNN-1))/NEURONS_PER_CORE_KNN)

#define NEURONS_PER_SUM_COUNTER 128

#define REMOTE_MESSAGE_BYTE_SIZE 8
#define LOCAL_MESSAGE_BYTE_SIZE 32

#define COUNTERS_PER_LAKEMONT (NEURONS_PER_CHIP_KNN/3)
#define SUM_COUNTERS ((COUNTERS_PER_LAKEMONT+NEURONS_PER_SUM_COUNTER-1)/NEURONS_PER_SUM_COUNTER)

// defined from the above parameters
#define SYN_MEM_LENGTH ((20+NEURONS_PER_CORE_KNN*8 + 63)/64)
#define NUM_KNN_CORES ((NEURONS_PER_CHIP_KNN + (NEURONS_PER_CORE_KNN-1))/NEURONS_PER_CORE_KNN)
#define NUM_ROUTING_CORES ((NEURONS_PER_CHIP_ROUTING + (NEURONS_PER_CORE_ROUTING-1))/NEURONS_PER_CORE_ROUTING)

#define TOKENS_PER_LOCAL_MESSAGE (LOCAL_MESSAGE_BYTE_SIZE >> 1)
#define TOKENS_PER_REMOTE_MESSAGE (REMOTE_MESSAGE_BYTE_SIZE >> 1)

#define STOP_COUNTER (COUNTERS_PER_LAKEMONT + (COUNTERS_PER_LAKEMONT+NEURONS_PER_SUM_COUNTER-1)/NEURONS_PER_SUM_COUNTER)

#define NUM_MESSAGES 128

#define GENERAL_ARRAY_SIZE 512
#endif