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

// Adapted from
// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

// Note: You may want to implement your own Spinner or implement custom callback
// queues Refer http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
// Also:
// https://answers.ros.org/question/53055/ros-callbacks-threads-and-spinning/
// Please consider your host cpu configuration if you need multiple threads

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <string>
#include "nxsdkhost.h"
#include "ros/ros.h"
#include "ros/transport_hints.h"
#include "std_msgs/Int32.h"

namespace ros_demo {
// ROS Topic which the PubSubProcess publishes and subscribes to
const char TOPIC[] = "example";
// Input CSP Channel which the Host Process writes to communicate with
// embedded snip
const char input_channel[] = "input";
// Feedback CSP Channel which the Host Process reads from to communicate with
// embedded snip
const char feedback_channel[] = "feedback";
// ROS Queue size for buffering messages
const int QUEUE_SIZE = 1000;

class PubSubProcess : public ConcurrentHostSnip {
  // ROS Node Handle
  std::unique_ptr<ros::NodeHandle> _node;
  // ROS Topic Publisher (Reads from feedback_channel and publishes to TOPIC)
  ros::Publisher _pub;
  // ROS Topic Subscriber (Subscribes to TOPIC and writes to input_channel)
  ros::Subscriber _sub;
  // Handshake to determine a loop is now complete, old data has been processed and new data has arrived
  bool _control_loop_complete;

 public:
  PubSubProcess() : _control_loop_complete(true) {
    // Write some initial data to input channel
    uint32_t data[1] = {0};
    writeChannel(input_channel, data, 1);

    // Initializes ROS
    int argc = 1;
    const char* argv[1] = {"PubSubProcess"};
    ros::init(argc, const_cast<char**>(argv), "pubsub");

    // Create the ROS Node Handle
    _node = std::make_unique<ros::NodeHandle>();

    // Subscribes to a ROS Topic "example" and registers a callback
    _sub = _node->subscribe(TOPIC, QUEUE_SIZE, &PubSubProcess::callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // Register to publish to ROS Topic "example"
    _pub = _node->advertise<std_msgs::Int32>(TOPIC, QUEUE_SIZE);
  }

  void run(std::atomic_bool& endOfExecution) override {
    // Set the rate of processing at 1000 Hz
    ros::Rate loop_rate(1000);

    // Loop to publish and subscribe
    while (ros::ok() && !endOfExecution) {
      if (publish()) {
        // Execute all callbacks till control loop is complete
        _control_loop_complete = false;
        while(!_control_loop_complete) {
            ros::spinOnce();
        }
        loop_rate.sleep();
      }
    }
  }

  // Callback method called by the Subscriber when it receives a message
  void callback(const std_msgs::Int32::ConstPtr& msg) {
    // Write to the input channel
    int32_t data[] = {msg->data};
    writeChannel(input_channel, data, 1);
    // Demonstrates logging using ROS_INFO
    ROS_INFO("Received: [%d]", msg->data);

    // Set control loop completion to true as new data has been read from topic
    _control_loop_complete = true;
  }

  // Reads from feedback_channel and publishes data to ROS Topic
  // Returns true if any messages were published. Otherwise, returns false
  bool publish() {
    int numMessagesToRead = probeChannel(feedback_channel);
    // Read an integer from feedback channel
    if (numMessagesToRead > 0) {
      uint32_t data[1] = {0};
      readChannel(feedback_channel, data, 1);

      // Create a ROS Message (http://wiki.ros.org/std_msgs)
      std_msgs::Int32 msg;
      msg.data = data[0];

      // Publish the message to the registered ROS Topic "example"
      _pub.publish(msg);

      // Demonstrates ROS Logging
      ROS_INFO("Sent: [%d]", msg.data);
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace ros_demo

using ros_demo::PubSubProcess;

// Each ConcurrentHostSnip is run within a thread
// If you have more threads on the host cpu, you can choose to create individual
// snips for publishers and subscribers
REGISTER_SNIP(PubSubProcess, ConcurrentHostSnip);
