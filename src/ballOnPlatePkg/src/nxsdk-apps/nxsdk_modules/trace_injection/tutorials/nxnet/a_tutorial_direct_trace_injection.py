# INTEL CORPORATION CONFIDENTIAL AND PROPRIETARY

# Copyright © 2019-2021 Intel Corporation.

# This software and the related documents are Intel copyrighted
# materials, and your use of them is governed by the express
# license under which they were provided to you (License). Unless
# the License provides otherwise, you may not use, modify, copy,
# publish, distribute, disclose or transmit  this software or the
# related documents without Intel's prior written permission.

# This software and the related documents are provided as is, with
# no express or implied warranties, other than those that are
# expressly stated in the License.

import os
import matplotlib as mpl        # For plotting without GUI

haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

import numpy as np
import nxsdk.api.n2a as nx
import matplotlib.pyplot as plt
from nxsdk.utils.plotutils import plotRaster
# import our trace injection class
from nxsdk_modules.trace_injection.src.direct_trace_injection import DirectTraceInjection



# # Direct Trace Injection
#
# For learning, a synapse only has access to its own parameters (weight, delay, tag) and its pre- and post-synaptic traces. Some applications may need an external signal to use during training, such as a global error signal. This tutorial shows how a snip can be used to overwrite postsynaptic traces with values provided from python.
#
# A snip will be used to overwrite trace values of the postsynaptic compartments just before learning executes. The list of new trace values will be provided by the superhost over a channel at every learning update.
#
# ![Direct Trace Injection](../figures/jupyter_snips.png)
#
# ## The steps in this tutorial are:
# 1. Setup the network as usual
# 2. Create an instance of the *direct_trace_injection* class, passing the constructor the compartment group to be overwritten
# 3. Compile the network as usual
# 4. Call the *setupSnips()* function of the *direct_trace_injection* instance to prepare the snips for execution
# 5. Start the board driver as usual
# 6. Call the *sendSnipInitialization()* function of the *direct_trace_injection* instance to send the initialization data to the snip
# 7. Run the board with the *aSync=True* argument to make the call non-blocking
# 8. At each learning epoch, call the *write_y()* function of the direct_trace_injection instance, passing a list of y trace values to be written to the compartment group
#

# ## 1. Setup the Network
#
# ### Setup the compartments
# In our case we will just create a single group of compartments called cx.
#
# However, if we wanted to overwrite traces of arbitrary compartments in a network, we can put those compartments together in a group (a compartment can belong to many groups).
#
# For example:
# ~~~~
# cx=net.createCompartmentGroup()
# cx.addCompartments(compartment1) #add existing compartment1 to the group
# cx.addCompartments(compartment2) #add existing compartment2 to the group
# cx.addCompartments(compartment3) #add existing compartment3 to the group
# cx.addCompartments(compartmentGroup1[n]) #add the nth compartment (index starts at 0) of compartmentGroup1 to the group
# ~~~~
#
# We can then modify the traces of all compartments in cx. Note that adding a compartment to a group does not change the network structure unless you explicitly connect something to that group or modify some properties of the group.


def setupNetwork(net, numTrainNeurons, numSteps, tEpoch):

    # Compartment group parameters
    vThMant = 50
    compartmentCurrentDecay = int(1/5*2**12)
    compartmentVoltageDecay = int(1/10*2**12)

    # Compartment prototype
    # We do not need to enable the trace computation (enableSpikeBackprop) because we'll be explicitly overwriting the trace values
    # with our snip
    cxProto = nx.CompartmentPrototype(vThMant=vThMant,
                                      compartmentCurrentDecay=compartmentCurrentDecay,
                                      compartmentVoltageDecay=compartmentVoltageDecay
                                      )

    # Compartment group that we'll overwrite the traces of
    cx = net.createCompartmentGroup(size=numTrainNeurons,
                                    prototype=cxProto
                                    )


# ### Setup the learning rule and connection
#
# This example only uses the y1 trace, but y2 and y3 traces can also be used.
# Overwriting the y1 trace also requires overwriting the y2 and y3 traces. We cannot mix injected traces with traces computed by the compartment itself.
#
# The learning rule here does not use x1 or x2, but there is no reason not to (try it).
# Since we do not use x1 or x2, we have not set up any x1 or x2 trace parameters.
#
# y1, y2, and y3 trace computation parameters do not need to be set because we will overwrite any computed values.
#
# y1 must always be positive, but using a bracketed term (y1-64) allows us to inject errors ranging from -64 to +63 by varying y1 from 0 to 127
# ~~~
# dw = 'u0*(y1-64)-u0*w'
# ~~~
# has the effect
# ~~~
# w = 'u0*(y1-64)'
# ~~~
# i.e. previous values of w do not affect the current value

    dw = 'u0*(y1-64)-u0*w'

    lrTrain = net.createLearningRule(dw=dw,
                                     tEpoch=tEpoch
                                     )

    connProto = nx.ConnectionPrototype(enableLearning=1,
                                       learningRule=lrTrain
                                       )


# ### Create Spike Generators
#
# We use a constant spike rate input to the compartment group

    # inter spike interval
    ISI = 10

    # Create a presynaptic input spike generator for the learning synapses
    sgDataIn = net.createSpikeGenProcess(numPorts=1)

    spikeTimes = list(range(0, numSteps, ISI))

    sgDataIn.addSpikes(0, spikeTimes)

    # define the synaptic weights connecting the spike generator to the post synaptic-learning compartment
    weight = np.array([np.linspace(-64, 0, numTrainNeurons, endpoint=True)]).T

    connLearn = sgDataIn.connect(cx,
                                 prototype=connProto,
                                 weight=weight
                                 )


# ## Configure Probes

# In[5]:

    # see the compartment state
    (uProbe, vProbe, sOutProbe) = cx.probe(
        [nx.ProbeParameter.COMPARTMENT_CURRENT, nx.ProbeParameter.COMPARTMENT_VOLTAGE, nx.ProbeParameter.SPIKE])

    # see the state of the synapse
    wLearnProbe = connLearn.probe(nx.ProbeParameter.SYNAPSE_WEIGHT)
    tLearnProbe = connLearn.probe(nx.ProbeParameter.SYNAPSE_TAG)


# ## 2. Create an instance of the DirectTraceInjection class
#

# In[6]:

    dti = DirectTraceInjection(net=net,
                               compartmentGroup=cx,
                               enableY1=1,
                               enableY2=0,
                               enableY3=0)


# pack probes
    probes = []
    probes.append(uProbe)
    probes.append(vProbe)
    probes.append(sOutProbe)
    probes.append(wLearnProbe)
    probes.append(spikeTimes)

    return dti, probes


if __name__ == "__main__":

    numTrainNeurons = 10
    numSteps = 1000
    tEpoch = 50

    net = nx.NxNet()
    dti, probes = setupNetwork(net, numTrainNeurons, numSteps, tEpoch)

    compiler = nx.N2Compiler()
    board = compiler.compile(net)
    dti.setupSnips(board=board)

    # ## 5. Start the board driver
    board.start()

    # ## 6. Send the snip initialization data
    #
    # The dti class has already determined internally which physical cores/compartments/traces to modify
    dti.sendSnipInitialization()

    # ## 7. Run the board with the non-blocking "async=True" argument
    board.run(numSteps, aSync=True)

    # ## 8. At each epoch, send a list of trace values to write
    #
    # If y2 and y3 were enabled when we initialized dti, then we would need to send lists of trace values for y2 and y3 as well.

    # how many learning epochs will there be in the run?
    numLearningEpochs = int(numSteps/tEpoch)

    # a simple list of values
    # initialize to the weight to some values
    y1ValueArray = np.linspace(-64, 0, numTrainNeurons, endpoint=True)
    # how much to increment the weight by on each epoch
    y1Increment = -2*y1ValueArray/numLearningEpochs

    # for each learning epoch
    for timestep in range(numLearningEpochs):
        y1ValueArray = y1ValueArray+y1Increment  # increment the weights
        # offset by 64 and convert to an integer list
        y1Values = [int(ii) for ii in (64+y1ValueArray).tolist()]
        # write the values to the compartment traces
        dti.writeY(y1Values=y1Values)

    # ## Cleanup and visualize

    # Finish and disconnect
    board.finishRun()
    board.disconnect()

    # ## Visualize results
    #
    # Our learning rule has the effect w=y1, so we should see the weight increase as we write increasingly large y1 trace values to the compartment. When the weight gets large enough, the input spike train causes the post-synaptic compartment to spike.

    uProbe = probes[0]
    vProbe = probes[1]
    sOutProbe = probes[2]
    wLearnProbe = probes[3]
    inputSpikeTimes = probes[4]

    # Plot compartment current, voltage and spikes
    numPlots = 5
    plotNumber = 0
    figDetector = plt.figure(1, figsize=(18, numPlots*3))

    plotNumber = plotNumber+1
    ax = plt.subplot(numPlots, 1, plotNumber)
    uProbe.plot()
    plt.title('Learning Compartment Current')
    limits = ax.get_xlim()

    plotNumber = plotNumber+1
    ax = plt.subplot(numPlots, 1, plotNumber)
    vProbe.plot()
    plt.title('Learning Compartment Voltage')

    plotNumber = plotNumber+1
    ax = plt.subplot(numPlots, 1, plotNumber)
    plotRaster([inputSpikeTimes])
    plt.title('Learning Compartment Input Spikes')
    ax.set_xlim(limits)

    plotNumber = plotNumber+1
    ax = plt.subplot(numPlots, 1, plotNumber)
    sOutProbe.plot()
    plt.title('Learning Compartment Output Spikes')
    ax.set_xlim(limits)

    plotNumber = plotNumber+1
    ax = plt.subplot(numPlots, 1, plotNumber)
    for ii in range(len(wLearnProbe)):
        wLearnProbe[ii][0].plot()
    plt.title('Train Synaptic Weight')

    # Show or save plot.
    if haveDisplay:
        plt.show()
    else:
        fileName = "%s.png" % os.path.splitext(os.path.basename(__file__))[0]
        print("No display available, saving to file " + fileName + ".")
        figDetector.savefig(fileName)
