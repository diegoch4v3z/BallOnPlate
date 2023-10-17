# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY
#
# Copyright © 2019-2021 Intel Corporation.
#
# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.
#
# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

"""Tutorial demonstrates implementing a simple control loop using rospy
We use the python rospy from rospkg module here to publish and subscribe to a topic

Prerequisites:

    0. Follow pre-requisites in tutorial_23_host_snips_with_ros_integration
    1. ROS Imports: `pip3 install rospkg pyyaml catkin-pkg`
    2. source /opt/ros/noetic/setup.bash
    3. Start ros core on a separate terminal: `source /opt/ros/noetic/setup.bash; roscore`

How to observe throughput from terminal:

 - rostopic hz /example
   - display publishing rate of topic
 - rostopic echo /example
   - print messages to screen

ROSPY API Documentation: http://docs.ros.org/api/rospy/html/

Scenario:

In this tutorial, we create the following pipeline:

InputProcess ------------------------>input channel---------------> EmbeddedSnip runMgmt (read from channel)
 subscribes to TOPIC example                                               |
        ^                                                                  |
        |                                                                  V
        |                                                                Update
        |                                                                  |
        |                                                                  |
 publishes to TOPIC example                                                V
FeedbackProcess  <-------------------feedback channel<------------- EmbeddedSnip runMgmt (write to channel)

Note: For lower latency, host snips could be explored.
      See tutorials/nxcore/tutorial_23_host_snips_with_ros_integration
"""
from abc import ABC
from threading import Thread

import os
import atexit

import rospy
from std_msgs.msg import Int32
from typing import List

from nxsdk.graph.channel import Channel
from nxsdk.arch.n2a.n2board import N2Board
from nxsdk.graph.processes.phase_enums import Phase

# ROS Topic for communication
ROS_TOPIC = "example"

# Number of timesteps to run
NUM_TIMESTEPS = 1000

# End of run marker
END_OF_RUN = False


class AbstractROSProcess(ABC):
    """Abstract class to represent a ROS Process communicating with embedded snips"""

    def __init__(self, topic: str, channel: Channel):
        """Setup"""
        pass

    def __call__(self, *args, **kwargs):
        """Read/Write to Embedded channels. Publish/Subscribe to topic"""
        pass

    def cleanup(self):
        """Close any pending resources"""
        pass


class InputProcess(AbstractROSProcess):
    """Subscribes to ROS TOPIC and writes the received data to the input channel to be read by embedded snip"""

    def __init__(self, topic: str, channel: Channel):
        self.inputChannel = channel

        # Write an initial value to to the input channel
        self.inputChannel.write(1, [0])

        # start subscriber after the network so that you don't hit any issues in sending data
        # before the network has been created
        # Every time a message is received, it will invoke the __call__ method
        self.subscriber = rospy.Subscriber(
            name=topic, data_class=Int32, callback=self, queue_size=1000)

    def __call__(self, *args, **kwargs):
        # This method is invoked every time a message is received on ROS TOPIC
        value = args[0].data
        rospy.loginfo_throttle(1, "Subscriber received: [" + str(value) + "]")

        # Write this data back to channel
        self.inputChannel.write(1, [value])

    def cleanup(self):
        """No cleanups necessary"""
        pass


class FeedbackProcess(AbstractROSProcess):
    """Publishes to ROS TOPIC and reads the feedback data from the feedback channel written by embedded snip"""

    def __init__(self, topic: str, channel: Channel):
        self.feedbackChannel = channel

        # Create the ROS Rpublisher
        self.publisher = rospy.Publisher(
            name=topic,
            data_class=Int32,
            queue_size=1000,
            latch=True)

        # Create a thread which will invoke the __call__ to read from the feedback channel
        # and publish to the ROS Topic
        self.thread = Thread(target=self)
        self.thread.start()

    def __call__(self, *args, **kwargs):
        global END_OF_RUN
        try:
            rate = rospy.Rate(1000)  # 1000hz
            while True:
                value = self.feedbackChannel.read(1)[0]
                if value < 1000:
                    # Publishing to ROS Topic
                    rospy.loginfo_throttle(
                        1, "Publishing value: [" + str(value) + "]")
                    self.publisher.publish(value)
                    rate.sleep()
                else:
                    rospy.loginfo(
                        "End of run. Completed Timesteps {}".format(value))
                    END_OF_RUN = True
                    break
        except Exception as e:
            rospy.logwarn(str(e))

    def cleanup(self):
        """Join with the publisher thread"""
        self.thread.join()


def ros_spin() -> None:
    """Spin forever till end of run is not true"""
    global END_OF_RUN
    rospy.logwarn("Logs are throttled to only print every second")
    while END_OF_RUN is False:
        rospy.rostime.wallsleep(0.5)


def setupNetwork() -> (N2Board, Channel, Channel):
    """Setup the Network as mentioned in the scenario"""

    # Instantiate the N2Board
    board = N2Board(1, 1, [1], [[10]])

    # Creating another snip to do management whenever do_run_mgmt condition is
    # true
    cFilePath = os.path.dirname(os.path.realpath(__file__)) + "/runmgmt.c"
    includeDir = os.path.dirname(os.path.realpath(__file__))
    funcName = "run_mgmt"
    guardName = "do_run_mgmt"
    embeddedProcess = board.createSnip(
        phase=Phase.EMBEDDED_MGMT,
        cFilePath=cFilePath,
        includeDir=includeDir,
        funcName=funcName,
        guardName=guardName)

    # Create a channel named input for sending data to embedded snip (mgmt phase)
    inputChannel = board.createChannel(
        name=b'input', messageSize=4, numElements=10000)
    # Connecting input channel from superhost to embeddedProcess making it
    # send channel
    inputChannel.connect(None, embeddedProcess)

    # Create a channel named feedback for receiving data from embedded snip (mgmt phase)
    feedbackChannel = board.createChannel(
        name=b'feedback', messageSize=4, numElements=10000)
    # Connecting feedback channel from embeddedProcess to superhost making it
    # receive channel
    feedbackChannel.connect(embeddedProcess, None)

    return board, inputChannel, feedbackChannel


class Cleanup:
    """Cleanup class to disconnect at exit"""

    def __init__(self, board: N2Board, processes: List[AbstractROSProcess]):
        self.board = board
        self.processes = processes

    def __call__(self, *args, **kwargs):
        """disconnect after ros is done"""
        try:
            for process in self.processes:
                process.cleanup()

            board.finishRun()
            board.disconnect()
        except BaseException:
            # Ignore any exceptions
            pass


if __name__ == "__main__":
    # Init the ROS Node so that publishers and subscribers can be created
    rospy.init_node('pubsub', anonymous=True)

    # Setup the network, create processes and channels. Compile the network
    board, inputChannel, feedbackChannel = setupNetwork()

    # Start the board. This setups the channels on the host and embedded ready
    # for communication
    board.start()

    # Write an initial value into the inputChannel and start the subscriber
    inputProcess = InputProcess(topic=ROS_TOPIC, channel=inputChannel)

    # Start the publisher and listen on the feedback channel
    feedbackProcess = FeedbackProcess(topic=ROS_TOPIC, channel=feedbackChannel)

    # Register for cleanup
    c = Cleanup(board=board, processes=[inputProcess, feedbackProcess])
    atexit.register(c)

    board.run(numSteps=NUM_TIMESTEPS, aSync=True)

    # Spin till end of run
    ros_spin()
