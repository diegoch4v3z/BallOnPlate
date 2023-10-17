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

#ifndef INITIALIZATION_H
#define INITIALIZATION_H
#include "array_sizes.h"
#include "nxsdk.h"

//for rewiring
extern int global_synFmtIndex[num_rewiring_cores]; //indicates which synFmtIndex to modify
extern int global_coreId[num_rewiring_cores]; //indicates which coreId to modify
extern int global_connection_order[num_rewiring_cores][max_connection_order_length]; //the order in which connections should be made
extern int global_connection_order_length[num_rewiring_cores]; //how long is the order for each core
extern int global_connection_index[num_rewiring_cores]; //where we currently are in the connection_order list


void initParams(runState *s);


#endif