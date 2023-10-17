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

#include "nxsdk.h"
typedef unsigned char uint8_t;
typedef signed char int8_t;

typedef struct ScanLines {
  uint8_t i1;
  uint8_t j1;
  uint8_t i2;
  uint8_t j2;
  int8_t m;
} ScanLine;

void GenScanLines(ScanLine *lines, int *numLines);
void GenDVSSpike(uint8_t *dvsspike, int *img1, ScanLine *lines, int numLines1, int N0, int pp);
int do_spiking(runState *s);
void run_spiking(runState *s);
