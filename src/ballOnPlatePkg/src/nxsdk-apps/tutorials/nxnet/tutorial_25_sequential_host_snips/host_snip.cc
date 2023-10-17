/*
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

#include <unistd.h>
#include <experimental/optional>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include "nxsdkhost.h"

// As we need to read the precomputed axon file, this file will get
// over-written with the correct file_location at runtime. This could
// be very well hard-coded if you know the exact path to the file.
static std::string precomputed_axons_file = "";  // NOLINT

// normal_distribution holds the current spike distribution in effect
// Initially it is set to null. But after the first run, it gets set
// to normal_distribution(mean=0, stddev=0.5). For subsequent runs,
// the mean is incremented so that the distribution changes.
std::default_random_engine generator;
using std::experimental::nullopt;
using std::experimental::optional;
using optional_distribution = optional<std::normal_distribution<double>>;
optional_distribution normal_distribution = nullopt;

// Preferred Axon Offset during a run
// (to vary the distribution to target certain axon)
static int preferred_axon_offset = -1;

// The Spike Injection Process sends the axon ids. Embedded Process
// injects the real spikes.
class SelectAxonForSpikeInjection : public PreExecutionSequentialHostSnip {
 private:
  // Channel used to communicate between host and lakemont
  std::string channel = "axon_info";
  // Vector of Axon Ids which have been pre-computed and stored in a file
  std::vector<uint32_t> axons;

 public:
  SelectAxonForSpikeInjection() {
    // Loads the precomputed_axons_file and stores the axon ids
    std::ifstream infile(precomputed_axons_file);
    int axon;
    while (infile >> axon) axons.push_back(axon);
  }

  virtual void run(uint32_t timestep) {
    // If normal distribution has not been initialized (which is the first run)
    // write axon ids into channel in a round robin fashion
    if (!normal_distribution) {
      int input_axon_id = axons[(timestep - 1) % axons.size()];
      writeChannel(channel.c_str(), &input_axon_id, 1);
    } else {
      // Get the next offset from the distribution
      int preferred_axon = round(normal_distribution.value()(generator));

      // Get the input axon id and write it to channel
      // to which spike should be injected
      if (preferred_axon >= 0 && preferred_axon < 10) {
        int input_axon_id = axons[preferred_axon];
        writeChannel(channel.c_str(), &input_axon_id, 1);
      } else {
        int input_axon_id = axons[preferred_axon_offset];
        writeChannel(channel.c_str(), &input_axon_id, 1);
      }
    }
  }

  virtual std::valarray<uint32_t> schedule(
      const std::valarray<uint32_t>& timesteps) const {
    // Scheule this snip on every timestep
    return timesteps;
  }
};

class ChangeSpikeDistribution : public PostExecutionSequentialHostSnip {
 public:
  virtual void run(uint32_t timestep) {
    ++preferred_axon_offset;

    std::cout << std::endl << "Changing spike distribution at end of timestep " << timestep << std::endl;
    std::cout << "normal distribution(mean=" << preferred_axon_offset % 10 << ", stddev=0.5)" << std::endl << std::endl;
    // Update the distribution
    normal_distribution =
        std::normal_distribution<double>(preferred_axon_offset % 10, 0.5);
  }

  virtual std::valarray<uint32_t> schedule(
      const std::valarray<uint32_t>& timesteps) const {
    // Schedule this only at the very last step of any run
    // As this is a Post Sequential Host Snip, it will execute after all
    // embedded phases are done and network has completed executing the
    // current run
    return {timesteps[timesteps.size() - 1]};
  }
};

// Register the Snips
REGISTER_SNIP(SelectAxonForSpikeInjection, PreExecutionSequentialHostSnip);
REGISTER_SNIP(ChangeSpikeDistribution, PostExecutionSequentialHostSnip);
