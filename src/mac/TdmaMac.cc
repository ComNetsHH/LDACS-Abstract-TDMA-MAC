//
// Copyright (C) 2013 OpenSim Ltd
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//
// author: Zoltan Bojthe
//

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../scheduler/TdmaScheduler.h"
#include "TdmaMac.h"
#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolGroup.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/linklayer/acking/AckingMac.h"
#include "inet/linklayer/acking/AckingMacHeader_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/networklayer/contract/IInterfaceTable.h"

using namespace inet;
using namespace inet::physicallayer;

Define_Module(AbstractLdacsTdmaMac);

AbstractLdacsTdmaMac::AbstractLdacsTdmaMac()
{
}

AbstractLdacsTdmaMac::~AbstractLdacsTdmaMac()
{
    assignedSlotsSH.clear();
    assignedSlotsP2P.clear();
    cancelAndDelete(transmissionSelfMessageSH);
    cancelAndDelete(transmissionSelfMessageP2P);
    cancelAndDelete(ackTimeoutMsg);
}

void AbstractLdacsTdmaMac::initialize(int stage)
{
    LayeredProtocolBase::initialize(stage);
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        upperLayerInGateId = findGate("upperLayerIn");
        upperLayerOutGateId = findGate("upperLayerOut");
        lowerLayerInGateId = findGate("lowerLayerIn");
        lowerLayerOutGateId = findGate("lowerLayerOut");
        frameLength = par("frameLength");
        slotDuration = par("slotDuration");
        buildGraphIntervalSlots = par("buildGraphIntervalSlots");
        bitrate = par("bitrate");
        headerLength = par("headerLength");
        promiscuous = par("promiscuous");
        fullDuplex = par("fullDuplex");
        useAck = par("useAck");
        ackTimeout = par("ackTimeout");
        numRetries = par("numRetries");
        maxP2PLinks = par("maxP2PLinks");

        // Retrieve the MAC address of the current node
        IInterfaceTable *interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        InterfaceEntry *interfaceEntry = interfaceTable->findInterfaceByName("wlan0");

        cModule *radioModule = gate("lowerLayerOut")->getPathEndGate()->getOwnerModule();
        // hostModule is defined in MacProtocolBase
        // hostModule = findContainingNode(this);
        mobilityModule = check_and_cast<IMobility *>(hostModule->getSubmodule("mobility"));
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radio = check_and_cast<IRadio *>(radioModule);
        transmissionState = IRadio::TRANSMISSION_STATE_UNDEFINED;

        // Shared channel queue
        txQueue = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queue"));
        // Point-to-point channel queue
        txQueueP2P = check_and_cast<queueing::IPacketQueue *>(getSubmodule("queueP2P"));

        scheduler = getModuleFromPar<AbstractLdacsTdmaScheduler>(par("scheduler"), this);

        transmissionSelfMessageSH = new cMessage("transmission-SH");
        transmissionSelfMessageP2P = new cMessage("transmission-P2P");

        EV << "slotDuration: " << slotDuration << endl;
        EV << "frameLength: " << frameLength << endl;

        macDelaySHSignal = registerSignal("macDelaySH");
        macDelayP2PSignal = registerSignal("macDelayP2P");

       }
        else if (stage == INITSTAGE_LINK_LAYER) {
            radio->setRadioMode(fullDuplex ? IRadio::RADIO_MODE_TRANSCEIVER : IRadio::RADIO_MODE_RECEIVER);
            nodeMacAddress = interfaceEntry->getMacAddress();
            // Obtain nodeId by registering the client and intiating SH and P2P buffers with 0
            nodeId = scheduler->registerClient(this, 0, 0, mobilityModule, nodeMacAddress);
            if (useAck) {
                ackTimeoutMsg = new cMessage("link-break");
            }
            // run time error as buildGraphIntervalSlots can not equal 0
            if (buildGraphIntervalSlots == 0) {
                throw cRuntimeError("The buildGraphIntervalSlots parameter should be larger than 0.");
            }
        }
}

void AbstractLdacsTdmaMac::handleSelfMessage(cMessage *message)
{
    if (message == ackTimeoutMsg) {
        EV << "AckingMac: timeout: " << currentTxFrame->getFullName() << endl;
        if(currentTransmissionAttemps +1 > numRetries) {
            // packet lost
            emit(linkBrokenSignal, currentTxFrame);
            PacketDropDetails details;
            details.setReason(OTHER_PACKET_DROP);
            dropCurrentTxFrame(details);
            currentTransmissionAttemps = 0;
            EV << "AckingMac: Lost frame" << endl;
        }else {
            EV << "AckingMac: Retrying..." << endl;
            currentTransmissionAttemps++;
        }
    }
    else if(message == transmissionSelfMessageSH) {
        if (!txQueue->isEmpty()) {
            if(currentTxFrame == nullptr) {
                popTxQueue();
            }
            // Capture start of transmission time
            startTransmissionTimeSH = simTime();
            // Calculate MAC layer delay and store
            simtime_t macLayerDelaySH = startTransmissionTimeSH - headOfQueueTimeSH;
            EV_INFO << "SH MAC delay is: " << macLayerDelaySH << endl;
            emit(macDelaySHSignal, macLayerDelaySH);
            scheduler->recordTransmissionTimeSH(nodeId, startTransmissionTimeSH);
            startTransmitting();
            headOfQueueTimeSH = simTime();
            if(hasFutureGrantSH()) {
                simtime_t nextTransmissionSlotTime = getNextTransmissionSlotSH();
                scheduleAt(nextTransmissionSlotTime, transmissionSelfMessageSH);
            }
        }
    }
    else if(message == transmissionSelfMessageP2P) {
        if (!txQueueP2P->isEmpty()) {
            if(currentTxFrameP2P == nullptr) {
                popTxQueueP2P();
            }
            startTransmissionTimeP2P = simTime();
            simtime_t macLayerDelayP2P = startTransmissionTimeP2P - headOfQueueTimeP2P;
            EV_INFO << "P2P MAC delay is: " << macLayerDelayP2P << endl;
            emit(macDelayP2PSignal, macLayerDelayP2P);
            scheduler->recordTransmissionTimeP2P(nodeId, startTransmissionTimeP2P);
            startTransmittingP2P();
            headOfQueueTimeP2P = simTime();
            if(hasFutureGrantP2P()) {
                simtime_t nextTransmissionSlotTime = getNextTransmissionSlotP2P();
                scheduleAt(nextTransmissionSlotTime, transmissionSelfMessageP2P);
            }
        }
    }
    else {
        MacProtocolBase::handleSelfMessage(message);
    }
}

void AbstractLdacsTdmaMac::handleUpperPacket(Packet *packet)
{
    MacAddress dest = packet->findTag<MacAddressReq>()->getDestAddress();
    if (!dest.isBroadcast() && !dest.isMulticast() && !dest.isUnspecified()) { // unicast use a point-to-point channel
        if (txQueueP2P->isEmpty()) {
            headOfQueueTimeP2P = simTime();
        }
        EV_INFO << "Received an application unicast packet." << endl;
        txQueueP2P->pushPacket(packet);
        scheduler->reportBufferStatusP2P(nodeId, txQueueP2P->getNumPackets());
    }
    else { // use shared channel
        if (txQueue->isEmpty()) {
            headOfQueueTimeSH = simTime();
        }
        txQueue->pushPacket(packet);
        scheduler->reportBufferStatusSH(nodeId, txQueue->getNumPackets());
    }
}

void AbstractLdacsTdmaMac::handleMessageWhenDown(cMessage *message) {
    return;
}

void AbstractLdacsTdmaMac::acked(Packet *frame)
{
    Enter_Method_Silent();
    ASSERT(useAck);

    if (currentTxFrame == nullptr) {
        throw cRuntimeError("Unexpected ACK received");
    }

    EV_DEBUG << "AckingMac::acked(" << frame->getFullName() << ") is accepted\n";
    cancelEvent(ackTimeoutMsg);
    deleteCurrentTxFrame();
    scheduler->reportBufferStatusSH(nodeId, txQueue->getNumPackets());
    currentTransmissionAttemps = 0;
}

void AbstractLdacsTdmaMac::popTxQueueP2P() {
    if (currentTxFrameP2P != nullptr)
        throw cRuntimeError("Model error: incomplete transmission exists");
    ASSERT(txQueueP2P != nullptr); // Ensure the txQueueP2P is not null
    currentTxFrameP2P = txQueueP2P->popPacket();
    scheduler->reportBufferStatusP2P(nodeId, txQueueP2P->getNumPackets());
    take(currentTxFrameP2P); // Take ownership of the packet
}

void AbstractLdacsTdmaMac::startTransmittingP2P() {
    // if there's any control info, remove it; then encapsulate the packet
    MacAddress dest = currentTxFrameP2P->getTag<MacAddressReq>()->getDestAddress();
    Packet *msg = currentTxFrameP2P;
    if (useAck && !dest.isBroadcast() && !dest.isMulticast() && !dest.isUnspecified()) {    // unicast
        msg = currentTxFrameP2P->dup();
        scheduleAt(simTime() + ackTimeout, ackTimeoutMsg);
    }
    else
        currentTxFrameP2P = nullptr;

    encapsulate(msg);

    // send
    EV << "Starting transmission of " << msg << endl;
    radio->setRadioMode(fullDuplex ? IRadio::RADIO_MODE_TRANSCEIVER : IRadio::RADIO_MODE_TRANSMITTER);
    sendDown(msg);
}

void AbstractLdacsTdmaMac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) {
    EV <<  "AbstractLdacsTdmaMac: Ignoring radio initiated transmission" << endl;
}

simtime_t AbstractLdacsTdmaMac::getNextTransmissionSlotSH() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = ceil(currentTime / slotDuration);
    double currentSlotIndex = currentGlobalSlotIndex % buildGraphIntervalSlots;
    int startIndex = currentSlotIndex +1;

    // Iterate over the assigned slots and see if there is a grant
    if (!assignedSlotsSH.empty()) {
        for (int slot : assignedSlotsSH) {
            if (slot >= startIndex) {
                return (currentGlobalSlotIndex + slot - currentSlotIndex) * slotDuration;
            }
        }
    }

    throw cRuntimeError("AbstractLdacsTdmaMac thinks we have a next grant but can't find it");
    return 0;
}

simtime_t AbstractLdacsTdmaMac::getNextTransmissionSlotP2P() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = ceil(currentTime / slotDuration);

    // Iterate over the assigned slots and see if there is a grant
    if (assignedSlotP2P != -1) {
        if (assignedSlotP2P == currentGlobalSlotIndex) {
            return assignedSlotP2P * slotDuration;
        }
    }

    throw cRuntimeError("AbstractLdacsTdmaMac tamara thinks we have a next grant but can't find it");
    return 0;
}

simtime_t AbstractLdacsTdmaMac::getFirstSlotInNextFrameSH() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = (int)(currentTime / slotDuration);
    int currentFrameIndex = currentGlobalSlotIndex / buildGraphIntervalSlots;
    int nextFrameindex = currentFrameIndex + 1;

    if (!assignedSlotsSH.empty()) {
        int firstSlot = assignedSlotsSH[0]; // Access the first slot assigned
        return (nextFrameindex * buildGraphIntervalSlots + firstSlot) * slotDuration;
    }
    throw cRuntimeError("AbstractLdacsTdmaMac thinks we have a grant but can't find it");
    return 0;
}

simtime_t AbstractLdacsTdmaMac::getFirstSlotInNextFrameP2P() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = (int)(currentTime / slotDuration);
    int currentFrameIndex = currentGlobalSlotIndex / buildGraphIntervalSlots;
    int nextFrameindex = currentFrameIndex + 1;

    if (!assignedSlotsP2P.empty()) {
        int firstSlot = assignedSlotsP2P[0]; // Access the first slot assigned
        return (nextFrameindex * buildGraphIntervalSlots + firstSlot) * slotDuration;
    }
    throw cRuntimeError("AbstractLdacsTdmaMac thinks we have a grant but can't find it");
    return 0;
}

bool AbstractLdacsTdmaMac::hasGrantSH() {
    if (!assignedSlotsSH.empty()) {
        return true;
    }
    return false;
}

bool AbstractLdacsTdmaMac::hasGrantP2P() {
    if (assignedSlotP2P != -1) {
        return true;
    }
    return false;
}

bool AbstractLdacsTdmaMac::hasFutureGrantSH() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = ceil(currentTime / slotDuration);
    double currentSlotIndex = currentGlobalSlotIndex % buildGraphIntervalSlots;
    int startIndex = currentSlotIndex +1;

    EV << "CurrentSlotIndex: " << currentSlotIndex << " (Globally: " << currentGlobalSlotIndex << ")" << endl;

    // Iterate over the assigned slots
    if (!assignedSlotsSH.empty()) {
        for (int slot : assignedSlotsSH) {
            if (slot >= startIndex) {
                // Found the first slot greater than or equal to startIndex
                EV << "Next grant in SH channel at slot " << slot << endl;
                return true;
            }
        }
    }
    EV << "No future grant in SH channel, will wait until next scheduling" << endl;
    return false;
}

bool AbstractLdacsTdmaMac::hasFutureGrantP2P() {
    double currentTime = simTime().dbl();
    int currentGlobalSlotIndex = ceil(currentTime / slotDuration);

    EV << "CurrentSlotIndex Globally: " << currentGlobalSlotIndex << endl;

    // Iterate over the assigned slots
    if (assignedSlotP2P != -1) {
        if (assignedSlotP2P == currentGlobalSlotIndex + 1) {
            // Found the first slot greater than or equal to startIndex
            EV << "Next grant in P2P channel at global slot " << assignedSlotP2P << endl;
            return true;
        }
    }
    EV << "No future grant in P2P channel, will wait until next scheduling" << endl;
    return false;
}

void AbstractLdacsTdmaMac::setScheduleSH(vector<int> slots) {
    Enter_Method_Silent();
    assignedSlotsSH = slots;

    if(transmissionSelfMessageSH->isScheduled()) {
        cancelEvent(transmissionSelfMessageSH);
    }
    if(hasGrantSH()) {
        simtime_t nextTransmissionTime = getFirstSlotInNextFrameSH();
        EV_INFO << hostModule->getFullName() << " next transmission time in the SH channel: " << nextTransmissionTime << "s." << endl;
        scheduleAt(nextTransmissionTime, transmissionSelfMessageSH);
    }
}

void AbstractLdacsTdmaMac::setScheduleP2P(int slot) {
    Enter_Method_Silent();
    assignedSlotP2P = slot;

    if(transmissionSelfMessageP2P->isScheduled()) {
        cancelEvent(transmissionSelfMessageP2P);
    }
    if(hasGrantP2P()) {
        // simtime_t nextTransmissionTime = getFirstSlotInNextFrameP2P();
        simtime_t nextTransmissionTime = getNextTransmissionSlotP2P();
        EV_INFO << hostModule->getFullName() << " next transmission time in the P2P channel: " << nextTransmissionTime << "s." << endl;
        scheduleAt(nextTransmissionTime, transmissionSelfMessageP2P);
    }
}

MacAddress AbstractLdacsTdmaMac::getHeadOfQueueMacP2P() {
    // EV_INFO << "Queue size before accessing head packet: " << txQueueP2P->getNumPackets() << endl;
    if (!txQueueP2P->isEmpty()) {
        auto headPacket = txQueueP2P->getPacket(0); // Retrieves the first packet in the queue without removing it
        MacAddress dest = headPacket->getTag<MacAddressReq>()->getDestAddress();
        // EV_INFO << "Dest MAC: " << dest << endl;
        EV_INFO << "Queue size after accessing head packet: " << txQueueP2P->getNumPackets() << endl;

        return dest; // Return the destination MAC address
    } 
    else {
        return MacAddress::UNSPECIFIED_ADDRESS; // Or handle this case as needed
    }
}

bool AbstractLdacsTdmaMac::queueIsEmptyP2P() {
    if (txQueueP2P->isEmpty()) {
        return false; 
    } 
    else {
        return true; // Or handle this case as needed
    }
}