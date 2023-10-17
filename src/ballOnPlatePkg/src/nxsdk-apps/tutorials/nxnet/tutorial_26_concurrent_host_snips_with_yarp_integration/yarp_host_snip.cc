/*
INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

Copyright © 2019-2021 Intel Corporation.

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

// Adapted from http://www.yarp.it/yarp_cmake_hello.html

#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include "nxsdkhost.h"

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;
using BP = BufferedPort<Bottle>;

const int NUM_CX = 4;
const int MAX_NUM_SPIKES = 14;

namespace yarp_globals {
// Static globals (Construct on First Use Idiom)
// Avoids static initialization fiasco and is thread-safe
Network& g_yarp_network() {
  // global yarp network
  static Network* yarp = new Network();
  return *yarp;
}
}  // namespace yarp_globals

namespace yarp_demo {
// Instantiate the globals in this namespace
auto& yarp = yarp_globals::g_yarp_network();
const char yarp_in[] = "/cx/in";
const char yarp_out[] = "/cx/out";

void check_success(bool result, const std::string& name) {
  // Checks if the result is true while opening a port with name
  if (!result) {
    std::cerr << "Failed to create port on name: " << name << std::endl;
    std::cerr << "Maybe you need to start a nameserver (run 'yarpserver')"
              << std::endl;
    exit(EXIT_FAILURE);
  }
}

// InputProcess which writes to EmbeddedSnip via input channel and reads from
// YARP port /cx/in
class HostProcess : public ConcurrentHostSnip {
  const std::string inputChannel = "input";
  const std::string feedbackChannel = "feedback";
  BP inputPort;
  BP outputPort;
  int cxId;
  int numSpikes;
  int32_t timeStamp;
  bool firstTimeStep{true};
  int cntrIdx[NUM_CX] = {0};
  uint64_t attempt = 0;

 public:
  HostProcess() {
    // Opening the input port
    bool result = inputPort.open(yarp_in);
    check_success(result, yarp_in);

    // Opening the output port
    result = outputPort.open(yarp_out);
    check_success(result, yarp_out);

    // Make a connection between the output port and the input port
    if (!yarp.connect(yarp_out, yarp_in)) {
      std::cerr << "Ports could not be connected" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

 public:
  void run(std::atomic_bool& endOfExecution) override {
    int sentinel = 0;  // Flag to tell us if its end of execution
    while (endOfExecution == false) {
      if (firstTimeStep == true) {
        // For the first timestep don't read anything
        firstTimeStep = false;
      } else {
        // For all other timesteps, read the value from inputPort
        Bottle* in = inputPort.read();
        if (in == nullptr) {
          std::cerr << "Failed to read message" << std::endl;
          exit(EXIT_FAILURE);
        }
        cxId = in->pop().asInt32();
        std::cout << "Cx read from input port is : " << cxId << std::endl;
        writeChannel(inputChannel.c_str(), &cxId, 1);
        cxId++;

        // Reading numSpikes from feedback channel
        // numSpikes indicates number of Cx that have spiked and
        // should be read from the channel
        readChannel(feedbackChannel.c_str(), &numSpikes, 1);
        if (numSpikes) {
          // Reading the timestamp when the Cx spiked
          readChannel(feedbackChannel.c_str(), &timeStamp, 1);
          // Reading the ids of cx which spiked
          readChannel(feedbackChannel.c_str(), cntrIdx, numSpikes);
        }

        while (numSpikes) {
          std::cout << "Time Stamp is : " << timeStamp << std::endl;
          std::cout << "Cx Id : " << cntrIdx[numSpikes - 1] << std::endl;
          numSpikes--;
        }

        // Reading the snitel value to check if it is end of execution
        // sentinel value of -1 indicated end of execution
        readChannel(feedbackChannel.c_str(), &sentinel, 1);
        if (sentinel == -1) {
          std::cout << "Execution Over, Stop execution of Host Snip "
                    << std::endl;
          break;
        }
      }
      std::cout << "Cx In which spike will be inserted is : " << cxId
                << std::endl;
      // Sending the CxId via outputPort
      Bottle& out = outputPort.prepare();
      out.clear();
      out.addInt32(cxId % NUM_CX);
      outputPort.write(true);
    }
  }
};

}  // namespace yarp_demo

using yarp_demo::HostProcess;
REGISTER_SNIP(HostProcess, ConcurrentHostSnip);
