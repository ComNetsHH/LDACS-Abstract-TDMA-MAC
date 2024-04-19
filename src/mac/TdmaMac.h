// The LDACS Abstract TDMA MAC models an abstract LDACS air-to-air TDMA-based MAC protocol.
// Copyright (C) 2024  Musab Ahmed, Konrad Fuger, Koojana Kuladinithi, Andreas Timm-Giel, Institute of Communication Networks, Hamburg University of Technology, Hamburg, Germany
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef __INET_TDMA_MAC_H
#define __INET_TDMA_MAC_H

#include "inet/common/INETDefs.h"
#include "inet/queueing/contract/IPacketQueue.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/linklayer/acking/AckingMac.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/LayeredProtocolBase.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"
using namespace inet;
using namespace std;


class AbstractLdacsTdmaScheduler;


/** @brief
 * Implementation of the MAC layer
 *
 *    @author Musab Ahmed, Konrad Fuger, TUHH ComNets
 *    @date February 2024
 */
class AbstractLdacsTdmaMac : public AckingMac
{
    protected:
        // Simulation signals
        simsignal_t macDelaySHSignal;
        simsignal_t macDelayP2PSignal;

        // Basic MAC properties
        MacAddress nodeMacAddress;                ///< MAC address of current node.
        IMobility *mobilityModule = nullptr;      ///< Reference to the mobility module.
        AbstractLdacsTdmaScheduler* scheduler = nullptr; ///< Reference to the scheduler instance.
        Packet *currentTxFrameP2P = nullptr; ///< Currently transmitted frame in P2P channel
        int currentTransmissionAttemps = 0; ///< Counter to control retransmissions

        // Transmission queues
        queueing::IPacketQueue *txQueueP2P = nullptr; ///< Queue for P2P unicast messages.

        // Schedule and slot information
        vector<int> assignedSlotsSH;               ///< Slots assigned for SH communication.
        vector<int> assignedSlotsP2P;              ///< Slots assigned for P2P communication.
        int assignedSlotP2P;                       ///< Single slot assigned for P2P communication.

        // MAC layer identifiers and settings
        int nodeId;                                ///< ID of this MAC layer as obtained by the scheduler.
        double slotDuration;                       ///< Duration of a single slot.
        int frameLength;                           ///< Number of slots per frame.
        int buildGraphIntervalSlots;               ///< Interval (in slots) to rebuild the connectivity graph.
        int numRetries;                            ///< Maximum number of retransmissions.
        int maxP2PLinks;                           ///< Maximum number of usable P2P links.

        // MAC delay measurement
        simtime_t headOfQueueTimeSH;               ///< Timestamp when a packet was enqueued for SH.
        simtime_t startTransmissionTimeSH;         ///< Start time of current transmission in SH.
        simtime_t headOfQueueTimeP2P;              ///< Timestamp when a packet was enqueued for P2P.
        simtime_t startTransmissionTimeP2P;        ///< Start time of current transmission in P2P.

        // Self-messages for handling transmission
        cMessage *transmissionSelfMessageSH = nullptr;    ///< Self message for SH transmission.
        cMessage *transmissionSelfMessageP2P = nullptr;   ///< Self message for P2P transmission.

        // Initialization and message handling methods
        void initialize(int stage) override;
        virtual void handleUpperPacket(Packet *packet) override;
        virtual void handleMessageWhenDown(cMessage *message) override;
        virtual void handleSelfMessage(cMessage *message) override;
        virtual void acked(Packet *frame) override; ///< Callback function for another MAC instance to acknowledge a frame 

        // MAC Logic
        void popTxQueueP2P(); 
        void startTransmittingP2P(); 
        void receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details) override; ///< Overwritten function to prohibit the radio from causing transmissions
        simtime_t getNextTransmissionSlotSH(), getNextTransmissionSlotP2P();
        simtime_t getFirstSlotInNextFrameSH(), getFirstSlotInNextFrameP2P();
        bool hasGrantSH(), hasGrantP2P();
        bool hasFutureGrantSH(), hasFutureGrantP2P();

    public:
        // Interface Functions
        void setScheduleSH(vector<int> slots);
        void setScheduleP2P(int slot);
        MacAddress getHeadOfQueueMacP2P(); ///< This function return the MAC header with the destination address
        bool queueIsEmptyP2P();
    
        // Constructor and destructor
        AbstractLdacsTdmaMac();
        virtual ~AbstractLdacsTdmaMac();
};

#endif // __INET_TDMA_MAC_H