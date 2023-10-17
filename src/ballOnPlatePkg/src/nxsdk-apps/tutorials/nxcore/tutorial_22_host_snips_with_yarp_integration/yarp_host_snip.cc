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
#include <iostream>
#include <string>
#include "nxsdkhost.h"

using yarp::os::Bottle;
using yarp::os::BufferedPort;
using yarp::os::Network;
using BP = BufferedPort<Bottle>;

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
const char yarp_in[] = "/hello/in";
const char yarp_out[] = "/hello/out";

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
// YARP port /hello/in
class InputProcess : public PreExecutionSequentialHostSnip {
  const std::string channel = "input";
  BP port;

 public:
  InputProcess() {
    bool result = port.open(yarp_in);
    check_success(result, yarp_in);
  }

  virtual void run(uint32_t timestep) {
    uint32_t data[1] = {0};
    if (timestep == 1) {
      // For the first timestep, write 0 onto channel
      data[0] = 0;
    } else {
      // For all other timesteps, read the value from inPort
      Bottle* in = port.read();
      if (in == nullptr) {
        std::cerr << "Failed to read message" << std::endl;
        exit(EXIT_FAILURE);
      }
      data[0] = in->pop().asInt32();
    }
    writeChannel(channel.c_str(), data, 1);
  }

  virtual std::valarray<uint32_t> schedule(
      const std::valarray<uint32_t>& timesteps) const {
    // Schedule on all timesteps
    return timesteps;
  }
};

// FeedbackProcess which reads from EmbeddedSnip via feedback channel and writes
// to YARP port /hello/out
class FeedbackProcess : public PostExecutionSequentialHostSnip {
  const std::string channel = "feedback";
  BP port;

 public:
  FeedbackProcess() {
    bool result = port.open(yarp_out);
    check_success(result, yarp_out);
    // Make a connection between the output port and the input port
    // All PreExecution snips are constructed before PostExecution Snips
    // (See nxsdkhost.h SNIP_TYPE enum for construction order)
    if (!yarp.connect(yarp_out, yarp_in)) {
      std::cerr << "Ports could not be connected" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  virtual void run(uint32_t timestep) {
    // Read an integer from feedback channel
    uint32_t data[1] = {0};
    readChannel(channel.c_str(), data, 1);

    // prepare a message to send via outPort
    Bottle& out = port.prepare();
    out.clear();
    out.addInt32(data[0]);
    port.write(true);
  }

  virtual std::valarray<uint32_t> schedule(
      const std::valarray<uint32_t>& timesteps) const {
    // Schedule on all timesteps
    return timesteps;
  }
};

}  // namespace yarp_demo

using yarp_demo::InputProcess;
using yarp_demo::FeedbackProcess;

REGISTER_SNIP(InputProcess, PreExecutionSequentialHostSnip);
REGISTER_SNIP(FeedbackProcess, PostExecutionSequentialHostSnip);
