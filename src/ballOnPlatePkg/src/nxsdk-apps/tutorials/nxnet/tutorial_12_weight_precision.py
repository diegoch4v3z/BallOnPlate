"""
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
"""

# -----------------------------------------------------------------------------
# Tutorial: tutorial_12_weight_precision.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces synaptic weight precision. The weight precision
# is controlled by the number of wgtbits in synapseFmt (numWeightBits in
# ConnectionProtoype). We configure four connections each with a different
# synaptic weight. We illustrate the use of the synaptic numWeightBits to adjust
# the precision.

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

# For plotting without GUI
from copy import copy
import matplotlib.pyplot as plt
import nxsdk.api.n2a as nx
import os
import matplotlib as mpl
haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')

# Nx API

# plt is used for graphical displays


# Define a function to setup the network

def setupNetwork(net):

    # Create a compartment prototype with the following parameters:
    # biasMant: Configure bias mantissa.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 1*(2^6) = 64.
    # biasExp: Configure bias exponent.  Actual numerical bias is
    #   biasMant*(2^biasExp) = 1*(2^6) = 64.
    # vThMant: Configuring voltage threshold value mantissa. Actual numerical
    #   threshold is vThMant*(2^6) = 10*2^6 = 640
    # functionalState: The setting to PHASE_IDLE = 2 allows the compartment to
    #   be driven by a bias current alone.  See user guide for all possible
    #   phases.
    compartment_prototype1 = nx.CompartmentPrototype(biasMant=1,
                                                     biasExp=6,
                                                     vThMant=10,
                                                     functionalState=2,
                                                     compartmentCurrentDecay=1)

    # Create a compartment in the network using the prototype
    compartment1 = net.createCompartment(compartment_prototype1)

    # Create four other compartments in the network with default values
    compartment_prototype2 = nx.CompartmentPrototype(compartmentCurrentDecay=1)
    compartment2 = net.createCompartment(compartment_prototype2)
    compartment3 = net.createCompartment(compartment_prototype2)
    compartment4 = net.createCompartment(compartment_prototype2)
    compartment5 = net.createCompartment(compartment_prototype2)

    # Configure connection between compartment 1 and other compartments with
    # varying weights and weight precision.
    #
    # The weight used for spike accumulation can be specified by up to 8 bits.
    # For the first connection, the synaptic weight of 20 is used.
    #
    # Note on Synapse Format (nxcore implementation)
    #
    # Each logical core supports 15 different types of synapse formats where
    # each synpase format is a unique combination of numWeightBits, numDstBits,
    # compressionMode, signMode etc. Validation errors are flagged if the limit of
    # of 15 is exceeded per core.
    #
    # The synaptic format details the decoding configuration needed to
    # interpret the compressed synaptic data from SYNAPSE_MEM.

    # -----------------------------------------------------------------------
    # The numWeightBits field specifies the number of significant bits used for
    # the synaptic weight value. A value of -1 (7 using nxcore synapsefmt) maps to 8 bits,
    # thus keeping the full precision of the synaptic weight value.
    # Note, if the signMode is 1 (mixed), then the sign bit
    # is included in the weight value, thereby losing one more LSB.
    # The general mapping is the following:
    #      wgtShift = 8 - numWeightBits + isMixed
    #      wgt = (wgt >> wgtShift) << wgtShift
    # We configure 4 bits of precision for the first connection.
    # For this example, the first synapse has a synaptic weight of 20.
    # We want only the 4 most significant bits,
    # i.e. 0001 0100 --> 0001 0000 (16).
    #
    # -----------------------------------------------------------------------

    # -----------------------------------------------------------------------
    # The numDstBits (idxBits in nxcore API) field is the bit width of the compartment index value
    # ("idx") stored in synapse_mem that maps to the actual compartment
    # index CIdx stored in each synapse. CIdx is calculated by the
    # following: CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    # Valid values for numDstBits (idxBits) are 0..7 which map
    # to 0,6,7,8,9,10,11,12. Here, idxBits of 1 maps to 6.
    # -----------------------------------------------------------------------

    # -----------------------------------------------------------------------
    # Compression indicates how the synapses are compressed.
    # 0: sparse
    # 3: dense (shared index) uncompressed
    # -----------------------------------------------------------------------

    connectionProtoype1 = nx.ConnectionPrototype(weight=20, numWeightBits=4, compressionMode=0,
                                                 signMode=2)
    net._createConnection(compartment1, compartment2, connectionProtoype1)

    # connectionProtoype2 is almost identical to connectionProtoype1 but differs in weight and weight precision
    connectionProtoype2 = copy(connectionProtoype1)
    connectionProtoype2.weight = 16
    # numWeightBits of -1 signifies using full weight precision. Valid values are (-1, 0, 1, 2, 3, 4, 5, 6, 8)
    connectionProtoype2.numWeightBits = -1
    net._createConnection(compartment1, compartment3, connectionProtoype2)

    # -----------------------------------------------------------------------
    # For the connections below, we also change the signMode
    # The signMode controls the sign of the synaptic weight and tag.
    # 1: mixed - no shared sign bit (included in each weight value)
    # 2: excitatory shared sign bit (0/+)
    # 3: inhibitory shared sign bit (1/-)
    # -----------------------------------------------------------------------

    # We configure 6 bits of precision for the third connection.
    # This connection has a synaptic weight of 10. For this synapseFmt we will
    # configure the mixed signMode (fanoutType), i.e. the sign bit is included
    # in the weight value. Therefore, we need to specify an extra signficant bit
    # to get a final weight value of 8.
    # For example, without the signed bit included when we
    # set numWeightBits to 5 we lose the 3 LSBs, 0000 1010 --> 0000 1000 (8).
    # However, when we configure mixed signMode we lose another LSB (4 total)
    # due to the inclusion of the sign bit, i.e. 0000 1010 --> 0 000 0000 (0).
    # Thus, in order to include the signed bit in the weight value (mixed fanout)
    # and get 8 as our final weight value we need to specify 6 bits, i.e.
    # 0000 1010 lose 2 LSB for precision --> 0000 1000 and 1 more LSB
    # for the sign bit --> 0 000 1000 (8).

    connectionProtoype3 = copy(connectionProtoype1)
    connectionProtoype3.weight = 10
    connectionProtoype3.numWeightBits = 6
    connectionProtoype3.signMode = 1
    net._createConnection(compartment1, compartment4, connectionProtoype3)

    connectionProtoype4 = copy(connectionProtoype1)
    connectionProtoype4.weight = 8
    # numWeightBits of -1 signifies using full weight precision. Valid values are (-1, 0, 1, 2, 3, 4, 5, 6, 8)
    connectionProtoype4.numWeightBits = -1
    connectionProtoype4.signMode = 1
    net._createConnection(compartment1, compartment5, connectionProtoype4)

    # Add Probes
    probes = []

    # Create both COMPARTMENT_CURRENT and COMPARTMENT_VOLTAGE Probes for compartment1
    # compartment1 should show the current and voltage decay driven by bias only
    # Create a compartment probe to probe the compartment states: compartment current(U) and compartment voltage(V)
    # probeConditions=None implies default probe conditions will be used for each probe
    compartment1Probe = compartment1.probe([nx.ProbeParameter.COMPARTMENT_CURRENT,
                                            nx.ProbeParameter.COMPARTMENT_VOLTAGE],
                                           probeConditions=None)
    probes.append(compartment1Probe)

    # For all other compartments, we just monitor the COMPARTMENT_CURRENT probe parameter
    for compartment in [compartment2, compartment3, compartment4, compartment5]:
        compartmentProbe = compartment.probe([nx.ProbeParameter.COMPARTMENT_CURRENT],
                                             probeConditions=None)
        probes.append(compartmentProbe)

    return probes


if __name__ == "__main__":

    # Create a network
    net = nx.NxNet()

    # Setup the network
    probes = setupNetwork(net)

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    net.run(100)
    net.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------
    COMPARTMENT_CURRENT = 0
    COMPARTMENT_VOLTAGE = 1
    uProbes = [probe[COMPARTMENT_CURRENT] for probe in probes]
    # COMPARTMENT_VOLTAGE probe monitor was added only for first compartment
    vProbe = probes[0][COMPARTMENT_VOLTAGE]

    # Since there are no incoming spikes and noise is disabled by default
    # u remains constant at 0.
    fig1 = plt.figure(10)
    plt.subplot(1, 2, 1)
    uProbes[0].plot()
    plt.title('u0')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')
    # v increases due to the bias current. Upon
    # reaching the threshold of 640, the compartment resets to
    # 0 (please refer to the refractory delay tutorial for further
    # explanation of why the v plot looks like it is resetting to 128 instead
    # of 0). Since there is no refractory period, the voltage immediately
    # begins to increase again.
    plt.subplot(1, 2, 2)
    vProbe.plot()
    plt.title('v0')
    plt.xlabel('Time')
    plt.ylabel('Membrane voltage')

    # Synapse with synaptic weight of 20, and 4 bits of precision, i.e.
    # effectively a synaptic weight of 16.
    fig2 = plt.figure(11)
    plt.subplot(2, 1, 1)
    uProbes[1].plot()
    plt.title('wgt 20 bits 4 --> wgt 16')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 16 and full 8 bits of precision.
    # This u plot is identical to uProbes[1].
    plt.subplot(2, 1, 2)
    uProbes[2].plot()
    plt.title('wgt 16')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 10 with mixed fanout and 6 bits of precision,
    # i.e. effectively a synaptic weight of 8.
    fig3 = plt.figure(12)
    plt.subplot(2, 1, 1)
    uProbes[3].plot()
    plt.title('wgt 10 bits 6 and mixed --> wgt 8')
    plt.ylabel('Membrane current')

    # Synapse with synaptic weight of 8 and full 8 bits precision.
    # This u plot is idential to uProbes[3]
    plt.subplot(2, 1, 2)
    uProbes[4].plot()
    plt.title('wgt 8')
    plt.xlabel('Time')
    plt.ylabel('Membrane current')

    if haveDisplay:
        plt.show()
    else:
        fileName1 = "tutorial_12_fig10.png"
        fileName2 = "tutorial_12_fig11.png"
        fileName3 = "tutorial_12_fig12.png"
        print("No display available, saving to files " + fileName1 +
              ", " + fileName2 + " and " + fileName3 + ".")
        fig1.savefig(fileName1)
        fig2.savefig(fileName2)
        fig3.savefig(fileName3)
