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
// author: Konrad Fuger
//

#ifndef __INET_TDMA_SCHEDULER_H
#define __INET_TDMA_SCHEDULER_H

#include "../mac/TdmaMac.h"
#include "inet/common/INETDefs.h"
#include "inet/queueing/contract/IPacketQueue.h"
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/common/MacAddress.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/linklayer/acking/AckingMac.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/mobility/contract/IMobility.h"
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <iomanip> 

using namespace inet;
using namespace std;

using NodeToSlotsMap = std::unordered_map<int, std::vector<int>>; // Type alias for an unordered map from integers to vectors of integers
using SlotToNodesMap = std::map<int, std::vector<int>>; // Type alias for a map from integers to vectors of integers

/** @brief The AbstractLdacsTdmaScheduler is a standalone module which handles the
 * assignment of radio resources to individual MAC layer instances.
 *
 *    @author Musab Ahmed, Konrad Fuger, TUHH ComNets
 *    @date February 2024
 */
class AbstractLdacsTdmaScheduler: public cSimpleModule
{
    protected:
        // Simulation signals
        simsignal_t scheduleSignal;
        simsignal_t utilizationSignal;
        simsignal_t nodeIdSignal; // New signal declaration

        // Scheduler properties
        int numNodes = 0;
        int slotIndex = 0;
        int maxP2PLinks;

        // Client information
        std::map<int, AbstractLdacsTdmaMac*> clients;
        std::map<int, inet::MacAddress> clientsMacAddress;
        std::map<int, inet::IMobility*> mobilityModules;
        map<int, int> bufferStatusSH;
        map<int, int> bufferStatusP2P;

        // Node and slot mapping
        std::unordered_map<int, int> nodeMapping; // Node ID to index mapping
        std::unordered_map<int, int> localToGlobalSlotMappingSH; // Local to global slot ID mapping for SH schedule

        // Slot and frame configurations
        std::vector<std::vector<int>> adjacencyMatrix;
        NodeToSlotsMap nodeToSlotsMapSH; // Assigned slots for each node in SH
        NodeToSlotsMap nodeToSlotsMapP2P; // Assigned slots for each node in P2P
        SlotToNodesMap slotToNodesMapSH; // Assigned nodes for each slot in SH
        SlotToNodesMap slotToNodesMapP2P; // Assigned nodes for each slot in P2P
        std::unordered_map<int, simtime_t> lastAssignedSH; // Last assignment time in SH
        std::unordered_map<int, simtime_t> lastAssignedP2P; // Last assignment time in P2P

        // Timing and interval settings
        double frameDuration;
        double slotDuration;
        double utilization = 0;
        int frameLength;
        double communicationRange;
        int buildGraphIntervalSlots; // Interval in number of slots to rebuild the graph
        double buildGraphDuration;
        int minReassignmentSlotsSH;
        int minReassignmentSlotsP2P;

        // Current slot and frame indices for scheduling
        int currentGlobalSlotIndex;
        int nextGlobalSlotIndex;
        int currentLocalSlotIndex;
        int nextLocalSlotIndex;
        double minReassignmentDurationSH;
        double minReassignmentDurationP2P;

        // Slot and frame timing information
        int nextFrameStartGlobalSlotIndex;
        double nextSlotStartTime;
        double nextFrameStartTime;

        // Self messages for triggering events
        cMessage* schedulingSHSelfMessage = nullptr;
        cMessage* schedulingP2PSelfMessage = nullptr;
        cMessage* slotSelfMessage = nullptr; // Message for slot scheduling
        cMessage* buildGraphMsg = nullptr; // Message to trigger graph building

        // Initialization and message handling
        void initialize(int stage) override;
        virtual void handleMessage(cMessage *message) override;

        // Scheduler logic methods
        virtual void assignSlotsSH();
        virtual void assignSlotsP2P();
        void createScheduleSH();
        void createScheduleP2P();
        virtual void updateSlotTimeInfo();
        virtual void initializeSHAssignment(); // Initialize variables and structures for SH slot assignment.
        virtual void initializeP2PAssignment(); // Initialize variables and structures for P2P slot assignment.
        
        // Timing and slot index management
        int getCurrentGlobalSlotIndex();
        int getNextGlobalSlotIndex();
        int getCurrentLocalSlotIndex();
        int getNextLocalSlotIndex();
        int getCurrentFrameStartGlobalSlotIndex();
        int getNextFrameStartGlobalSlotIndex();
        double getNextFrameStartTime();
        double getNextSlotStartTime();

        // Helper functions
        std::pair<std::vector<std::vector<int>>, std::unordered_map<int, int>> createAdjacencyMatrixAndNodeMapping();
        void buildGraph();
        std::vector<int> findNodesWithinOneAndTwoHops(int nodeId);
        std::string getHostName(int nodeId);
        SlotToNodesMap createSlotToNodesMap(const NodeToSlotsMap& nodeToSlotsMap);
        void printSlotAssignments(const SlotToNodesMap& slotToNodesMap);
        void printNodeSlotAssignments(const NodeToSlotsMap& nodeToSlotsMap);
        void printBufferStatus(const std::map<int, int>& buffer);
        int findLocalSlotIndex(int currentGlobalSlotIndex); // Find the corresponding local slot index for the current global slot index
        std::unordered_set<int> populateAvailableNodesSH(double slotStart);
        std::unordered_set<int> populateAvailableNodesP2P(double slotStart); // Populates the set of available nodes based on their eligibility and buffer status.
        bool checkIfSlotExistsInSH(int nodeId, int localSlotIndex);  // Check if the the node have slots assigned in SH schedules.
        bool checkIfSlotExistsInP2P(int nodeId, int globalSlotIndex);  // Check if the the node have slots assigned in P2P schedules.
        int findNodeIdByMac(MacAddress macAddress); // Retrieve the node ID from its MAC address 
        int selectRandomNode(const std::unordered_set<int>& availableNodes); // Takes a set of available node IDs and returns one of them selected at random.
        int countAssignmentsForSlot(const std::unordered_map<int, std::vector<int>>& map, int slotID);

    public:
        // Constructor and destructor
        AbstractLdacsTdmaScheduler();
        virtual ~AbstractLdacsTdmaScheduler();

        // Client registration and status reporting
        int registerClient(AbstractLdacsTdmaMac *mac, int statusSH, int statusP2P, inet::IMobility *mobilityModule, inet::MacAddress macAddress);
        void reportBufferStatusSH(int nodeId, int bufferStatus);
        void reportBufferStatusP2P(int nodeId, int bufferStatus);

        // Transmission time recording
        void recordTransmissionTimeSH(int nodeId, simtime_t transmissionTimeSH);
        void recordTransmissionTimeP2P(int nodeId, simtime_t transmissionTimeP2P);
};

#endif

