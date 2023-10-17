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
# Tutorial: tutorial_20_soma_homeostasis.py
# -----------------------------------------------------------------------------
#
# This tutorial introduces the homeostasis feature of the soma threshold and
# will demonstrate how it can be configured and how it works.
# Different examples show the possible solutions to change the membrane
# threshold of compartments.
#
# Description of the examples in the respective methods, which can be
# chosen in the main (comment in/out).
# Detailed description can be found above the plot sections.
#
# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------

from nxsdk.arch.n2a.compiler.tracecfggen.tracecfggen import TraceCfgGen
from nxsdk.graph.nxinputgen.nxinputgen import BasicSpikeGenerator
from nxsdk.arch.n2a.n2board import N2Board
import numpy as np
import matplotlib.pyplot as plt
import os
import matplotlib as mpl

haveDisplay = "DISPLAY" in os.environ
if not haveDisplay:
    mpl.use('Agg')
# plt is used for graphical displays

# N2Board module provides access to the neuromorphic hardware

# Input generation module used to insert a basic spike


# Define a function to setup the network for the generalHomeostasisExample
def setupNetworkGeneralHomeostasisExample():

    # -----------------------------------------------------------------------
    # Initialize board
    # -----------------------------------------------------------------------

    # Board ID
    boardId = 1

    # Number of chips
    numChips = 1

    # Number of cores per chip
    numCoresPerChip = [1]

    # Number of synapses per core
    numSynapsesPerCore = [[1]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with one compartment driven by input spikes
    # -----------------------------------------------------------------------

    # Voltage decay (decayV) is set to 1/16 and the current decay (decayU)
    # is set to 1/10 (2^12 factor is for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1 / 16 * 2 ** 12),
                                     decayU=int(1 / 10 * 2 ** 12))

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0
    # is used. The setting of phase0 = PHASE_IDLE = 2 allows the
    # compartment to be driven by an input spike.
    # See the user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # Initialize vthProfileCfg
    # Homeostasis needs to be enabled (=1) to get somaTraces.
    # The threshold of vth changes according to the formular:
    # beta * (max(a - aMax) + min(a - aMin)).
    # Beta is a scaling factor and a is the activity/soma trace, a
    # trace of the post synaptic spikes from this compartment.
    n2Core.vthProfileCfg[0].dynamicCfg.configure(enableHomeostasis=1,
                                                 beta=1,
                                                 aMin=20,
                                                 aMax=80)

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 1000*2^6 = 64000
    n2Core.somaState[0].configure(vth=1000)

    # Initialize dendriteTimeState
    n2Core.dendriteTimeState[0].tepoch = 1

    # Initialize somaTraceCfg
    # Configure pre trace behavior (trace decay time constant and the level
    # by which a trace gets incremented upon each spike). Since the register
    # encoding of this information is quite complex, we use a helper class
    # (TraceCfgGen) that generates the register configuration based on the time
    # constant (tau) and increment (spikeLevel) and writes it to the appropriate
    # registers.
    tcg = TraceCfgGen()
    tc = tcg.genTraceCfg(tau=16,
                         spikeLevelInt=40,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.somaTraceCfg[0])

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 1
    n2Core.synapseMap[0].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[0].synapseLen = targetSynapseLen
    n2Core.synapseMap[0].discreteMapEntry.configure()

    # Configure synapses with different synaptic delays

    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits. It is a normalized signed value.
    # Here, the synaptic weight of 256 is used. There are 6 bits used to
    # specify the synaptic delay offset, (i.e. 1 to 63 time step delay).
    # While the dendritic accumulator accumulates the spike instantly,
    # this will delay the accumulation in 'u' for the specified number
    # of time steps. For this synapse there is no delay.
    # Each core supports 15 different types of synapse formats, i.e. indices 1 to
    # 15 where 0 is invalid. In this example synapse format ID 1 is used.
    n2Core.synapses[0].CIdx = 0
    n2Core.synapses[0].Wgt = 255
    n2Core.synapses[0].synFmtId = 1

    # Configure synaptic format
    configureSynapseFmt(n2Core.synapseFmt[1], wgtExp=0, wgtBits=7, dlyBits=6, numSynapses=1, idxBits=1, compresion=3,
                        fanoutType=2)

    return board

# Define a function to setup the network for the adaptiveThresholdHomeostasis


def setupNetworkAdaptiveThresholdHomeostasis():
    # -----------------------------------------------------------------------
    # Initialize board
    # -----------------------------------------------------------------------

    # Board ID
    boardId = 1

    # Number of chips
    numChips = 1

    # Number of cores per chip
    numCoresPerChip = [1]

    # Number of synapses per core
    numSynapsesPerCore = [[1]]

    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)

    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -----------------------------------------------------------------------
    # Configure core with one compartment driven by input spikes
    # -----------------------------------------------------------------------

    # Voltage decay (decayV) is set to 1/16 and the current decay (decayU)
    # is set to 1/10 (2^12 factor is for fixed point implementation)
    n2Core.cxProfileCfg[0].configure(decayV=int(1 / 16 * 2 ** 12),
                                     decayU=int(1 / 10 * 2 ** 12))

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4)
    # Since i=0 is chosen for this example, cxMetaState[0] with phase0
    # is used. The setting of phase0 = PHASE_IDLE = 2 allows the
    # compartment to be driven by an input spike.
    # See the user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2)

    # Initialize vthProfileCfg
    # Homeostasis needs to be enabled (=1) to get somaTraces
    # The threshold of vth changes according to the formular:
    # beta * (max(a - aMax) + min(a - aMin)).
    # Beta is a scaling factor and a is the activity/soma trace, a
    # trace of the post synaptic spikes from this compartment.
    n2Core.vthProfileCfg[0].dynamicCfg.configure(enableHomeostasis=1,
                                                 beta=1,
                                                 aMin=2,
                                                 aMax=80)

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6) = 148*2^6 = 9472
    n2Core.somaState[0].configure(vth=500)

    # Initialize dendriteTimeState
    n2Core.dendriteTimeState[0].tepoch = 4

    # Initialize somaTraceCfg
    # Configure pre trace behavior (trace decay time constant and the level
    # by which a trace gets incremented upon each spike). Since the register
    # encoding of this information is quite complex, we use a helper class
    # (TraceCfgGen) that generates the register configuration based on the time
    # constant (tau) and increment (spikeLevel) and writes it to the appropriate
    # registers.
    tcg = TraceCfgGen()
    tc = tcg.genTraceCfg(tau=0,
                         spikeLevelInt=100,
                         spikeLevelFrac=0)
    tc.writeToRegister(n2Core.somaTraceCfg[0])

    # The parameter numUpdates controls how many compartment groups
    # will be serviced (4 compartments per compartment group).
    # Specifically, numUpdates = i means that i compartment groups
    # will be updated, which translates to compartments 0 to 4i-1
    # (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^6) = 10*(2^6) = 640.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[0].configure(
        bias=10,
        biasExp=6,
        vthProfile=0,
        cxProfile=0)

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    targetSynapsePtr = 0
    targetSynapseLen = 1
    n2Core.synapseMap[0].synapsePtr = targetSynapsePtr
    n2Core.synapseMap[0].synapseLen = targetSynapseLen
    n2Core.synapseMap[0].discreteMapEntry.configure()

    # Configure synapses with different synaptic delays

    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    # The synaptic weight used for spike accumulation can be specified
    # by up to 8 bits. It is a normalized signed value.
    # Here, the synaptic weight of 256 is used. There are 6 bits used to
    # specify the synaptic delay offset, (i.e. 1 to 63 time step delay).
    # While the dendritic accumulator accumulates the spike instantly,
    # this will delay the accumulation in 'u' for the specified number
    # of time steps. For this synapse there is no delay.
    # Each core supports 15 different types of synapse formats, i.e. indices 1 to
    # 15 where 0 is invalid. In this example synapse format ID 1 is used.
    n2Core.synapses[0].CIdx = 0
    n2Core.synapses[0].Wgt = 50
    n2Core.synapses[0].synFmtId = 1

    # Configure synaptic format
    configureSynapseFmt(n2Core.synapseFmt[1], wgtExp=2, wgtBits=7, dlyBits=6, numSynapses=1, idxBits=1, compresion=3,
                        fanoutType=2)

    return board

# Define a function to setup the network for the adaptiveThresholdMulticompartmentNeuron


def setupNetworkAdaptiveThresholdMulticompartmentNeuron():
    # -------------------------------------------------------------------------
    # Initialize board
    # -------------------------------------------------------------------------

    # N2Board ID
    boardId = 1
    # Number of chips
    numChips = 1
    # Number of cores per chip
    numCoresPerChip = [1]
    # Number of synapses per core
    numSynapsesPerCore = [[3]]
    # Initialize the board
    board = N2Board(boardId, numChips, numCoresPerChip, numSynapsesPerCore)
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]

    # -------------------------------------------------------------------------
    # Configure core with 4 compartments
    # -------------------------------------------------------------------------

    # configure ALIF Cell (detailed description of parameters inside the function)
    # This function configures the compartments cxCfg[0] and cxCfg[1] and the
    # respective synapses and axon connections.
    configureALIFCell(n2Core, compartmentIndex=0, synapseIndex=0, inputWeight=98, inputWgtExp=4, baseline_thr=0.01,
                      scale=10 ** 7, beta=1.8, tau_a=700, tau_m=20, vthProfileAux=0, vthProfile=1,
                      cxProfileAux=0, cxProfile=1)

    # Configure compartment 2 with tau_m=20 and pop
    n2Core.cxProfileCfg[2].configure(decayV=int(1 / 20 * 2 ** 12),
                                     decayU=int(2 ** 12 - 1),
                                     stackIn=2, joinOp=1)

    # use the same vthProfile and cxProfile to keep track of the threshold
    n2Core.cxCfg[2].configure(
        bias=0,
        biasExp=0,
        vthProfile=1,
        cxProfile=2)

    # Configure compartment 3 without adaptive decay for comparison
    n2Core.cxProfileCfg[3].configure(decayV=int(1 / 16 * 2 ** 12),
                                     decayU=int(2 ** 12 - 1))

    # use the same vthProfile for comparison
    n2Core.cxCfg[3].configure(
        bias=0,
        biasExp=0,
        vthProfile=1,
        cxProfile=3)

    # For a chosen compartment i, the associated cxMetaState register is
    # cxMetaState[floor(i/4)] and with the field phaseK where K = mod(i,4).
    # Since i=0,1 are chosen for this example, cxMetaState[0] with phase0
    # and phase1 and phase2 is used. The setting of phase0 = phase1 = phase2 = PHASE_IDLE = 2
    # allows the compartment 0, 1 and 2 to be driven by a bias current alone, spike gen or
    # with input spikes.
    # See user guide for all possible phases.
    n2Core.cxMetaState[0].configure(phase0=2, phase1=2, phase2=2, phase3=2)
    # The parameter numUpdates controls how many compartment groups will be
    # serviced (4 compartments per compartment group). Specifically,
    # numUpdates = i means that i compartment groups will be update, which
    # translates to compartments 0 to 4i-1 (i in range 1 to 256).
    n2Core.numUpdates.configure(numUpdates=1)

    # -------------------------------------------------------------------------
    # Configure axons
    # -------------------------------------------------------------------------

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    # Synapses [0] and [1] have been taken care of in configureALIFCell()
    n2Core.synapseMap[2].synapsePtr = 2
    n2Core.synapseMap[2].synapseLen = 1
    n2Core.synapseMap[2].discreteMapEntry.configure()

    # Configure synapses

    # synapses for compartment 0 and 1 are configured in configureALIFCell()
    # no synapse for compartment 2 needed! (just monitoring)

    # This synapse connects to compartment 3 (i.e. cxCfg[3]). (also just for monitoring)
    # synFmtId 2 refers to a synFmt which is set inside of configureALIFCell()
    n2Core.synapses[2].CIdx = 3
    n2Core.synapses[2].Wgt = 98
    n2Core.synapses[2].synFmtId = 2

    # Return the configured board
    return board


def configureALIFCell(n2Core, compartmentIndex=0, synapseIndex=0, inputWeight=98, inputWgtExp=4, baseline_thr=0.01,
                      scale=10**7, beta=1.8, tau_a=700, tau_m=20, vthProfileAux=None, vthProfile=None,
                      cxProfileAux=None, cxProfile=None):
    """
    This functions configurates a ALIF cell as Multicompartment neuron.
    Each ALIF cell uses two compartments and 2 synapses.
    The ALIF cell represents a simple LIF neuron with an adaptive
    threshold.

    :param n2Core:           The core for the ALIF Cell.

    :param compartmentIndex: Start index of the 2 compartments to be used.
                             Meaning compartmentIndex and compartmentIndex
                             + 1 will be used.

    :param synapseIndex:     Start index of the 2 needed synapses (input
                             synapse of the 2 compartments)

    :param inputWeight:      The weight of the input synapse for the main
                             compartment.

    :param inputWgtExp:      The exponent for the weight of the input
                             synapse of the main compartment.

    :param baseline_thr:     static threshold in V. This value will be
                             scaled by the parameter scale and then
                             approximated by x * 2^6.

    :param scale:            Scales the baseline_thr to get higher
                             precision.

    :param beta:             scaling factor of the adaptive threshold

    :param tau_a:            time constant of the adaptive threshold

    :param tau_m:            time constant of the membrane

    :param vthProfileAux:    The vthProfile to be used for the auxiliary
                             compartment. Should be reused. If not
                             provided, a new vthProfile[0] will be
                             created.

    :param vthProfile:       The vthProfile to be used for the main
                             compartment. Should be reused. If not
                             provided, a new vthProfile[1] will be
                             created.

    :param cxProfileAux:     The cxProfile to be used for the auxiliary
                             compartment. Should be reused. If not
                             provided, a new cxProfile[0] will be
                             created.

    :param cxProfile:        The cxProfile to be used for the main
                             compartment. Should be reused. If not
                             provided, a new cxProfile[1] will be
                             created.
    """

    # -------------------------------------------------------------------------
    # Configure core with 2 compartments (main and auxiliary)
    # -------------------------------------------------------------------------

    # The configuration below uses cxProfileCfg[compartmentIndex] and
    # vthProfileCfg[compartmentIndex] to configure the compartment.

    if vthProfileAux is None:
        vthProfileAux = 0

    if vthProfile is None:
        vthProfile = 1

    if cxProfileAux is None:
        cxProfileAux = 0

    if cxProfile is None:
        cxProfile = 1

    # Auxiliary compartment:

    # Voltage decay (decayV) is set to 1 / tau_a and the current decay (decayU)
    # is set to maximum 2^12 -1 (2^12 factor is for fixed point implementation)
    # stackOut is set to 1 to push the value of v onto the Stack.
    n2Core.cxProfileCfg[cxProfileAux].configure(decayV=int(1 / tau_a * 2 ** 12),
                                                decayU=int(2 ** 12 - 1),
                                                stackOut=1)

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6)
    # In this case the threshold should never be reached and v should
    # always be lower or equal zero
    n2Core.vthProfileCfg[vthProfileAux].staticCfg.configure(vth=10000)
    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(2^biasExp) = 0*(2^0) = 0.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 0 references cxProfileCfg[0].
    n2Core.cxCfg[compartmentIndex].configure(
        bias=0,
        biasExp=0,
        vthProfile=vthProfileAux,
        cxProfile=cxProfileAux)

    # The configuration below uses cxProfileCfg[1] and vthProfileCfg[1] to
    # configure the compartment 1 and compartment 2.

    # Voltage decay (decayV) is set to 1/ tau_m and the current decay (decayU)
    # is set to maximum 2^12 -1 (2^12 factor is for fixed point implementation)
    # This represents a simple LIF model.
    # stackIn is set to 2 to POP the value from stack. Since we are pushing
    # the value of v of compartment 0 on the stack, it will be poped.
    # joinOp is set to 1, which means ADD will be used for Join Operation
    # ADD will add the peeked value from the stack to the current v value
    # For other supported Join Operations see user guide.
    n2Core.cxProfileCfg[cxProfile].configure(decayV=int(1 / tau_m * 2 ** 12),
                                             decayU=int(2 ** 12 - 1),
                                             stackIn=1, joinOp=1)

    # calculate the threshold mantissa
    mantissa = int((baseline_thr * scale) / (2**6))

    # Configuring threshold value mantissa.  Actual numerical threshold is
    # vth*(2^6)
    n2Core.vthProfileCfg[vthProfile].staticCfg.configure(vth=1562)

    # Configure bias mantissa and exponent.  Actual numerical bias is
    # bias*(biasExp) = 0*(2^0) = 0.
    # vthProfile = 0 references vthProfileCfg[0].
    # cxProfile = 1 references cxProfileCfg[1].
    n2Core.cxCfg[compartmentIndex+1].configure(
        bias=0,
        biasExp=0,
        vthProfile=vthProfile,
        cxProfile=cxProfile)

    # -------------------------------------------------------------------------
    # Configure axons (main to aux) and synapses
    # -------------------------------------------------------------------------

    # Configure output axon (connects main compartment with auxiliary)

    targetCoreId = 4
    targetAxonId = compartmentIndex
    sourceCompartmentId = compartmentIndex + 1
    targetChipId = n2Core.parent.id
    n2Core.createDiscreteAxon(
        sourceCompartmentId, targetChipId, targetCoreId, targetAxonId)

    # Configure input axon

    # Setup the connection to the synapes. Similar to the axonMap above,
    # the targetSynapsePtr indexes into synapses and targetSynapseLen
    # denotes how many consecutive indices in synapses (starting from
    # targetSynapsePtr) are connected to the input axon.
    for loop in range(synapseIndex, synapseIndex+2):
        n2Core.synapseMap[loop].synapsePtr = loop
        n2Core.synapseMap[loop].synapseLen = 1
        n2Core.synapseMap[loop].discreteMapEntry.configure()

    # Configure synapses

    # Inhibitory

    # calculate weight for inhibitory synapse as f(tau_a, tau_m, beta)
    weightAux = beta * (1 / (tau_a*tau_m) - 1/(tau_a**2)) * scale

    # map this weight to the synapse weight
    # weight formular is: (Wgt) * 2^WgtExp * 2^6 !if fanoutType is 1, Wgt is floored to an even number!
    wgt = int(weightAux / (2**6))

    exp = 0
    for i in range(0, 8):
        if wgt % 2 == 0:
            exp += 1
            wgt = int(wgt / 2)
        else:
            break

    # This synapse connects to compartment 0 (i.e. cxCfg[0]).
    # Each core supports 15 different types of synapses, i.e. indices 1 to 15,
    # and index 0 is invalid. In this example synapse format ID 1 and 2 are
    # being used.
    n2Core.synapses[0].CIdx = compartmentIndex
    n2Core.synapses[0].Wgt = -wgt
    n2Core.synapses[0].synFmtId = 1

    # Excitatory
    # This synapse connects to compartment 1 (i.e. cxCfg[1]).
    n2Core.synapses[1].CIdx = compartmentIndex + 1
    n2Core.synapses[1].Wgt = inputWeight
    n2Core.synapses[1].synFmtId = 2

    # synFmt for the inhibitory synapse
    configureSynapseFmt(n2Core.synapseFmt[1], wgtExp=exp, wgtBits=7, dlyBits=1, idxBits=6, numSynapses=1, compresion=3,
                        fanoutType=3)
    # synFmt for the excitatory synapse
    configureSynapseFmt(n2Core.synapseFmt[2], wgtExp=inputWgtExp, wgtBits=7, dlyBits=6, numSynapses=1, idxBits=1,
                        compresion=0, fanoutType=2)


# Configure the synapse format synFmt for the synapses
def configureSynapseFmt(synapseFmt, wgtExp, wgtBits, dlyBits, numSynapses, idxBits, compresion, fanoutType):
    """

    :param synapseFmt:      The respective synapseFmt (n2Core.synapseFmt[x])
    :param wgtExp:          The wgtExp field allows for the additional up or down bit shifting of the
                            Wgt parameter for normalization purposes.

    :param wgtBits:         The wgtBits field specifies the number of significant bits used for the
                            Wgt value. A setting of 7 maps to 8 bits, thus keeping the full
                            precision of the Wgt value.

    :param dlyBits:         The dlyBits field specifies the number of significant bits used for
                            the synaptic delay value. Valid values are 0..6.

    :param numSynapses:     Setting numSynapses specifies a certain form of encoding.  The value of
                            63 does not mean 63 synapses, but instead is a special value to have the
                            number of synapses encoded in the prefix field of synapse_mem.  To
                            describe this another way, using 63 asks the compiler figure out how to
                            map synapses into synapse entries, but setting another value for
                            numSynapses mean that the user specifies the mapping.

    :param idxBits:         idxBits set to 1 maps to 6 bits.  This is the bit width of the
                            compartment index value ("idx") stored in synapse_mem that maps to the
                            actual compartment index CIdx stored in each synapse.  The mapping is
                            CIdx = idx * (CIdxMult+1) + CIdxOffset +cxBase
    :param compresion:      Selects how synapses are compressed. Compression setting of 3 means
                            dense. Other modes include sparse, run-length compression, zero-mask
                            compression, etc.

    :param fanoutType:      fanoutType of 1 indicates mixed fanout, i.e. no shared sign bit (included
                            in each weight value)
    """

    synapseFmt.wgtExp = wgtExp
    synapseFmt.wgtBits = wgtBits
    synapseFmt.dlyBits = dlyBits
    synapseFmt.numSynapses = numSynapses
    synapseFmt.idxBits = idxBits
    synapseFmt.compression = compresion
    synapseFmt.fanoutType = fanoutType


# Configures the spike generator
def configureSpikeGenerator(board, targetAxonList, spikeTimes):
    bs = BasicSpikeGenerator(board)

    for targetAxon in targetAxonList:
        for time in spikeTimes:
            chipId = 0
            coreId = 4
            axonId = targetAxon
            bs.addSpike(time, chipId, coreId, axonId)


def generalHomeostasisExample():
    """

    This example shows how the homeostasis feature is behaving with different
    input sequences and how the threshold trace is evolving. Therefore we
    configure 1 compartment driven by an input spike generator. We inject 5 spike
    trains with different frequencies one after another into the compartment. The
    minimum and maximum thresholds of the homeostasis are set to 20 and 80,
    respectively. As long as the soma trace/activity stays in these bounds the
    homeostasis does not change the spiking threshold. If the soma trace exceeds
    the maximum threshold, the homeostasis will increase the spiking threshold
    which leads to less spikes and thus a lower soma trace until it falls below
    the maximum bound. If the soma trace/activity falls under the lower bound,
    the homeostasis will decrease the threshold which leads to more spikes and
    thus a higher soma trace until it reaches the minimum bound again.
    """

    # Setup the network
    board = setupNetworkGeneralHomeostasisExample()

    # Get the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # configure Input Spikes to axon 0
    # Input rate of spikes, where the activity stays insiside the bounds
    spikeTimes = np.append(np.arange(1, 300, 25), np.arange(301, 800, 10))

    # Input rate of spikes, which is lower than before but higher than the start
    spikeTimes = np.append(spikeTimes, np.arange(801, 1300, 20))

    # Input rate of spikes, which is lower than the beginning
    spikeTimes = np.append(spikeTimes, np.arange(1301, 1800, 35))

    targetAxon = 0
    configureSpikeGenerator(board, [targetAxon], spikeTimes)

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    # cxState[i] contains the state of cxCfg[i].  The probe method takes
    # a list of compartment indices to obtain the data for the
    # chosen field, e.g. u or v.
    # somaState[i] can be used to obtain data for the soma trace (a).
    # The data for each compartment index is returned as an element of a
    # list.
    uProbes = mon.probe(n2Core.cxState, [0], 'u')[0]
    spikeProbes = mon.probe(n2Core.cxState, [0], 'spike')[0]
    aProbes = mon.probe(n2Core.somaState, [0], 'a')[0]
    vthProbes = mon.probe(n2Core.somaState, [0], 'vth')[0]

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(2000)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # The spike generator injects spikes to u, thus spikes get elicited.
    # Each spike increases the activity a by a value of 40. The threshold
    # vth is decreasing in the beginning since the activity starts at 0
    # and needs time to get in the bounds. As soon as the activity is
    # between aMin and aMax vth is stable and not changing. After the
    # fist sector the frequency of the input spikes is increased. Thus
    # the activity increases as it exceeds the upper bound aMax, the
    # threshold vth increases also. This leads to an inhibition and
    # the activity decreases again until it is inside the bounds.
    # Another stable state is reached until the third sector. In the
    # third sector the frequency of the input spikes is decreased but
    # still higher than in the first sector. The threshold vth begins to
    # decrease until another stable state. In the forth sector the input
    # frequency is even further decreased and now lower than in the first
    # sector. Thus, the stable state is lower than in the beginning. At
    # the last sector no input spikes are given. One more spike is
    # elicited as the threshold decreases to zero and there is some
    # voltage left in the membrane.
    #
    fig = plt.figure(1020)
    k = 1

    # For each compartment plot the u, spike, a and vth data

    xposition = [300, 800, 1300, 1800]
    plt.subplot(2, 2, k)
    uProbes.plot()
    plt.title('u')
    # visually seperate the input rate changes
    for xc in xposition:
        plt.axvline(x=xc, color='k', linestyle='--')
    k += 1
    plt.subplot(2, 2, k)
    spikeProbes.plot()
    plt.title('spikes')
    # visually seperate the input rate changes
    for xc in xposition:
        plt.axvline(x=xc, color='k', linestyle='--')
    k += 1
    plt.subplot(2, 2, k)
    aProbes.plot()
    plt.title('a')
    # visually seperate the input rate changes
    for xc in xposition:
        plt.axvline(x=xc, color='k', linestyle='--')
    k += 1
    plt.axhline(y=20, color='tab:orange', linestyle='-.', label="aMin")
    plt.axhline(y=80, color='r', linestyle='-.', label="aMax")
    plt.legend()
    plt.subplot(2, 2, k)
    vthProbes.plot()
    plt.title('vth')
    # visually seperate the input rate changes
    for xc in xposition:
        plt.axvline(x=xc, color='k', linestyle='--')
    k += 1

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_20_fig_ex1.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)


def adaptiveThresholdHomeostasis():
    """
    This example shows an approach to implement an adaptive threshold using
    the homeostasis feature. The goal is to keep a baseline threshold, while
    also having a general decay on the threshold and an increase of the
    threshold after each elicited spike.
    """

    # Setup the network
    board = setupNetworkAdaptiveThresholdHomeostasis()

    # Get the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # configure Input Spikes
    spikeTimes = [2500]
    targetAxon = 0
    configureSpikeGenerator(board, [targetAxon], spikeTimes)

    # --------------------------------------------------------------------
    # Configure probes
    # --------------------------------------------------------------------

    mon = board.monitor

    # cxState[i] contains the state of cxCfg[i].  The probe method takes
    # a list of compartment indices to obtain the data for the
    # chosen field, e.g. u or v.
    # somaState[i] can be used to obtain data for the soma trace (a).
    # The data for each compartment index is returned as an element of a
    # list.
    uProbes = mon.probe(n2Core.cxState, [0], 'u')[0]
    spikeProbes = mon.probe(n2Core.cxState, [0], 'spike')[0]
    aProbes = mon.probe(n2Core.somaState, [0], 'a')[0]
    vthProbes = mon.probe(n2Core.somaState, [0], 'vth')[0]

    # --------------------------------------------------------------------
    # Run
    # --------------------------------------------------------------------

    board.run(4500)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # Setup: The compartment has a bias which generates a spike every 40th
    # timestep. The activity is configurated with a timeconstant of 0,
    # meaning that every spike yields an impuls of 1 timestep in the
    # activity. Since aMax is set to 80 and the activity impulse weight
    # is set to 100, every impulse increases the threshold vth by 20.
    # aMin is set to 2, so every timestep without an activity impulse
    # the threshold gets decreased by 2.
    # The bias leads to an input activity which keeps the threshold around
    # 150. The threshold vth starts at 500, which keeps the compartment
    # from spiking and thus the threshold decreases until the bias is
    # enough to yield a spike.
    #
    # External stimulation: At timestep 2500 a spike from the spike
    # generator is produced. This spike drives the membrane voltage
    # over the threshold and elicited multiple spikes. Thus the vth
    # increases. After that it decays back to its baseline.
    #
    # With different values of aMin, aMax, beta and the bias the baseline
    # and the decay of vth can be adjusted.

    fig = plt.figure(1020)
    k = 1

    # For each compartment plot the u, spike, a and vth data

    plt.subplot(2, 2, k)
    uProbes.plot()
    plt.title('u')

    k += 1
    plt.subplot(2, 2, k)
    spikeProbes.plot()
    plt.title('spikes')

    k += 1
    plt.subplot(2, 2, k)
    aProbes.plot()
    plt.title('a')

    k += 1
    plt.axhline(y=2, color='tab:orange', linestyle='-.', label="aMin")
    plt.axhline(y=80, color='r', linestyle='-.', label="aMax")
    plt.legend()
    plt.subplot(2, 2, k)
    vthProbes.plot()
    plt.title('vth')

    k += 1

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_20_fig_ex2.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)


def adaptiveThresholdMulticompartmentNeuron():
    """
    This example shows an approach to implement an adaptive threshold using
    the multi compartment model. This approach does not use the homeostasis
    feature!
    An adaptive threshold is provided by decreasing the membrane potential
    directly, thus the gap between the static threshold changes.

    The approach uses two compartments. There is a decay compartment, which
    gets a negative input if the input compartment elicited a spike. The
    decay compartment slowly decays back to zero, having a high time
    constant. These values are pushed to the stack und popped and added by
    the input compartment, which integrates these values into his membrane
    potential.
    """

    # -------------------------------------------------------------------------
    # Configure network
    # -------------------------------------------------------------------------
    # Setup the network
    board = setupNetworkAdaptiveThresholdMulticompartmentNeuron()
    # Obtain the relevant core (only one in this example)
    n2Core = board.n2Chips[0].n2Cores[0]
    board.sync = True

    # configure Input Spikes (spike at ts 100 for axon 1 and 2
    spikeTimes = [100, 300, 600, 601]
    targetAxonList = [1, 2]
    configureSpikeGenerator(board, targetAxonList, spikeTimes)

    # -------------------------------------------------------------------------
    # Configure probes
    # -------------------------------------------------------------------------

    mon = board.monitor

    uProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'u')
    vProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'v')
    spikeProbes = mon.probe(n2Core.cxState, [0, 1, 2, 3], 'spike')

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------

    board.run(1000)
    board.disconnect()

    # --------------------------------------------------------------------
    # Plot
    # --------------------------------------------------------------------

    # Setup: 4 compartments, although 2 are just for monitoring reasons.
    # There is a main input compartment which receives the input and
    # generates the output spikes and there is a decay compartment.
    # The decay compartment receives an input spike everytime the input
    # compartment elicited a spike.
    # There are additionally another input compartment which does not
    # get an input and just shows the effective decay and an input
    # compartment which has no decay for comparison.
    #
    # At timestep 100 the input compartments get a spike from the spike
    # generator. The membrane potential reaches the threshold and the
    # input compartment generates a spike. This spike is also send to
    # the decay compartment, leading to an decrease in its membrane
    # voltage. The membrane voltage of the decay compartment an input
    # compartment are integrated in the input compartment.
    # This leads to an adaptive threshold which is increased everytime
    # the input compartment elicit a spike and decays slowly with a
    # different time constant.
    #
    # Another input spike is given at time step 300. However, the threshold
    # is not reached this time, because the membrane voltage is still
    # modified by the decay compartment - adaptive threshold.
    #
    # At time step 600 and 601 input spikes are given, which lead to
    # a spike and thus the decay compartment is decreased again.
    #
    # With different values of the synapse weight connection the input
    # and decay compartment and different time constants the threshold
    # and decaying time span can be adjusted.

    fig = plt.figure(1020)
    k = 1
    legendList = [["decay compartment"], ["input compartment"], ["effective decay"],
                  ["input compartment without decay"]]
    for j in range(0, 4):
        # For each compartment plot the u, v and spike data
        plt.subplot(4, 3, k)
        uProbes[j].plot()
        plt.title('u')
        k += 1
        plt.subplot(4, 3, k)
        vProbes[j].plot()
        plt.title('v')
        plt.legend(legendList[j])
        k += 1
        plt.subplot(4, 3, k)
        spikeProbes[j].plot()
        plt.title('spikes')
        k += 1

    if haveDisplay:
        plt.show()
    else:
        fileName = "tutorial_20_fig_ex3.png"
        print("No display available, saving to file " + fileName + ".")
        fig.savefig(fileName)


if __name__ == "__main__":

    # --------------------------------------------------------------------------
    # A general example that shows the homeostasis feature, how it is configured
    # and how it behaves with different input sequences.
    # --------------------------------------------------------------------------
    generalHomeostasisExample()

    # --------------------------------------------------------------------------
    # An example of implementing an adaptive threshold with a baseline, using
    # the homeostasis feature.
    # --------------------------------------------------------------------------
    # adaptiveThresholdHomeostasis()

    # --------------------------------------------------------------------------
    # An example of implementing an adaptive threshold without using the
    # homeostasis feature, but shows other ways to implement a dynamic threshold
    # --------------------------------------------------------------------------
    # adaptiveThresholdMulticompartmentNeuron()
