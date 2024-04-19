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

#include "TdmaScheduler.h"

Define_Module(AbstractLdacsTdmaScheduler);

AbstractLdacsTdmaScheduler::AbstractLdacsTdmaScheduler() {

}

AbstractLdacsTdmaScheduler::~AbstractLdacsTdmaScheduler() {
    cancelAndDelete(schedulingSHSelfMessage);
    cancelAndDelete(schedulingP2PSelfMessage);
}

void AbstractLdacsTdmaScheduler::initialize(int stage) {
    frameLength = par("frameLength");
    slotDuration = par("slotDuration");
    communicationRange = par("communicationRange").doubleValue(); // initialize as double
    buildGraphIntervalSlots = par("buildGraphIntervalSlots");
    minReassignmentSlotsSH = par("minReassignmentSlotsSH");
    minReassignmentSlotsP2P = par("minReassignmentSlotsP2P");
    maxP2PLinks = par("maxP2PLinks");
    frameDuration = slotDuration * frameLength;
    buildGraphDuration = slotDuration * buildGraphIntervalSlots;
    minReassignmentDurationSH = slotDuration * minReassignmentSlotsSH;
    minReassignmentDurationP2P = slotDuration * minReassignmentSlotsP2P;

    scheduleSignal = registerSignal("schedule");
    utilizationSignal = registerSignal("utilization");
    ///////////////////////////////////
    /// (record link access delay)
    nodeIdSignal = registerSignal("nodeId"); // Register the signal

    // Message triggers the global scheduling process for SH links.
    schedulingSHSelfMessage = new cMessage("schedulingSH");
    // Message triggers the global scheduling process for P2P links.
    schedulingP2PSelfMessage = new cMessage("schedulingP2P");
    // Message for monitoring or logging slot assignments at each slot time.
    slotSelfMessage = new cMessage("slot");
    // Message triggers the building or updating of the network graph for scheduling decisions.
    buildGraphMsg = new cMessage("BuildGraph");

    WATCH(utilization);

    // start first scheduling during simulation start
    if (buildGraphIntervalSlots == 0) {
        throw cRuntimeError("The buildGraphIntervalSlots parameter should be larger than 0.");
    } else {
        scheduleAt(buildGraphDuration - 0.5 * slotDuration, buildGraphMsg);
    }
    
    // start first SH scheduling during the last slot before the next buildGraphDuration
    scheduleAt(buildGraphDuration - 0.5 * slotDuration, schedulingSHSelfMessage);
    // start first P2P scheduling during the last slot
    scheduleAt(buildGraphDuration - 0.25 * slotDuration, schedulingP2PSelfMessage);
    if(par("monitorSchedule")) {
        scheduleAt(buildGraphDuration, slotSelfMessage);
    }
}

void AbstractLdacsTdmaScheduler::handleMessage(cMessage *message) {
    if(message == schedulingSHSelfMessage) {

        EV << "AbstractLdacsTdmaScheduler: Start scheduling SH trasnmission" << endl;
        createScheduleSH();
        // createScheduleP2P();
        // createSchedule();
        scheduleAt(simTime() + buildGraphDuration, schedulingSHSelfMessage);
    }
    else if (message == schedulingP2PSelfMessage) {
        EV << "AbstractLdacsTdmaScheduler: Start scheduling P2P trasnmission" << endl;
        createScheduleP2P();
        scheduleAt(simTime() + slotDuration, schedulingP2PSelfMessage);
    }
    else if (message == buildGraphMsg) {
        buildGraph(); // Call your method to build or update the graph

        // After building the graph, reschedule the message for the next interval
        if (buildGraphDuration == 0) {
            scheduleAt(simTime() + slotDuration, buildGraphMsg);  
        } else {
            scheduleAt(simTime() + buildGraphDuration, buildGraphMsg);  
        }
        
    } 
}

int AbstractLdacsTdmaScheduler::registerClient(AbstractLdacsTdmaMac *mac, int statusSH, int statusP2P, inet::IMobility *mobilityModule, MacAddress macAddress) {
    Enter_Method_Silent();
    int nodeId = numNodes++;

    bufferStatusSH.insert(make_pair(nodeId, statusSH));
    bufferStatusP2P.insert(make_pair(nodeId, statusP2P));
    clients.insert(make_pair(nodeId, mac));
    clientsMacAddress.insert(make_pair(nodeId, macAddress));

    // store mobility modules associated with node IDs
    mobilityModules.insert(make_pair(nodeId, mobilityModule));

    EV << "SH channel: Registered " << mac->getName() << " as Node #" << nodeId << " with buffer status: " << statusSH << endl;
    EV << "P2P channel: Registered " << mac->getName() << " as Node #" << nodeId << " with buffer status: " << statusP2P << endl;
    return nodeId;
}

void AbstractLdacsTdmaScheduler::reportBufferStatusSH(int nodeId, int bufferStatus) {
    Enter_Method_Silent();
    EV << "SH channel: " << getHostName(nodeId) << " reported a Buffer Status of " << bufferStatus << endl;
    bufferStatusSH[nodeId] = bufferStatus;
}

void AbstractLdacsTdmaScheduler::reportBufferStatusP2P(int nodeId, int bufferStatus) {
    Enter_Method_Silent();
    EV << "P2P channel: " << getHostName(nodeId) << " reported a buffer status of " << bufferStatus << endl;
    bufferStatusP2P[nodeId] = bufferStatus;
}

void AbstractLdacsTdmaScheduler::recordTransmissionTimeSH(int nodeId, simtime_t transmissionTimeSH) {
    Enter_Method_Silent(); 
    // Record the current transmission time for the given node ID
    lastAssignedSH[nodeId] = transmissionTimeSH;

    // EV << "Recorded transmission time in SH channel for " << getHostName(nodeId) << ": " << transmissionTimeSH << endl;
}

void AbstractLdacsTdmaScheduler::recordTransmissionTimeP2P(int nodeId, simtime_t transmissionTimeP2P) {
    Enter_Method_Silent(); 
    // Record the current transmission time for the given node ID
    lastAssignedP2P[nodeId] = transmissionTimeP2P;

    // EV << "Recorded transmission time in P2P channel for " << getHostName(nodeId) << ": " << transmissionTimeP2P << endl;
}

void AbstractLdacsTdmaScheduler::assignSlotsSH() {
    // Slot and frame timing information
    initializeSHAssignment();
    updateSlotTimeInfo();
    // Initialize mapping of global slot ID to local slot ID for this frame
    for (int localSlot = 0; localSlot < buildGraphIntervalSlots; ++localSlot) {
        localToGlobalSlotMappingSH[localSlot] = nextFrameStartGlobalSlotIndex + localSlot;
    }

    // Iterate through each slot
    for (int slot = 0; slot < buildGraphIntervalSlots; ++slot) {
        // Calculate the start time for this specific slot in the next frame
        double slotStartTime = nextFrameStartTime + (slot * slotDuration);
        std::unordered_set<int> availableNodes = populateAvailableNodesSH(slotStartTime);

        while (!availableNodes.empty()) {
            int selectedNodeId = selectRandomNode(availableNodes);
            // Assign the selected node to this slot
            nodeToSlotsMapSH[selectedNodeId].push_back(slot);
            // Decrement the buffer status for P2P
            if (--bufferStatusSH[selectedNodeId] <= 0) {
                // If the buffer is now empty, remove the node from the bufferStatusSH for this frame
                bufferStatusSH.erase(selectedNodeId);
            }
            availableNodes.erase(selectedNodeId); // Remove the selected node from available nodes

            // Remove 1-hop and 2-hop neighbors from available nodes to avoid interference
            std::vector<int> neighbors = findNodesWithinOneAndTwoHops(selectedNodeId);
            for (int neighborId : neighbors) {
                availableNodes.erase(neighborId);
            }
            recordTransmissionTimeSH(selectedNodeId, slotStartTime);
        }
    }
    slotToNodesMapSH = createSlotToNodesMap(nodeToSlotsMapSH);
    EV << "Assign slots for the shared channel." << endl;
    printSlotAssignments(slotToNodesMapSH);
    // Optionally, show the updated buffer status
    EV << "Updated Buffer Status SH:" << endl;
    printBufferStatus(bufferStatusSH);
}

void AbstractLdacsTdmaScheduler::assignSlotsP2P() {
    initializeP2PAssignment();
    updateSlotTimeInfo();

    if (nextLocalSlotIndex == -1) {
        // If we couldn't find a corresponding local slot index, it's likely an error or edge case
        throw cRuntimeError("Next Global slot index in P2P schedule does not exist in the SH schedule.");
        // EV_INFO << currentGlobalSlotIndex << "Next Global slot index in P2P schedule does not exist in the SH schedule." << endl;
    }

    // Reset for each new assignment
    std::set<int> assignedRecipientsForCurrentSlot;
    assignedRecipientsForCurrentSlot.clear();
    std::unordered_set<int> availableNodes = populateAvailableNodesP2P(nextSlotStartTime);
    int numberOfAssignedP2PLinks = 0;

    while (!availableNodes.empty() && numberOfAssignedP2PLinks < maxP2PLinks) {
        int selectedNodeId = selectRandomNode(availableNodes);
        // Get the MAC address of the head of queue packet's intended recipient
        auto macModule = clients[selectedNodeId];
        auto recipientMac = macModule->getHeadOfQueueMacP2P();
        int recipientId = findNodeIdByMac(recipientMac);

        bool txSlotExistsInSH = checkIfSlotExistsInSH(selectedNodeId, nextLocalSlotIndex);
        bool rxSlotExistsInSH = checkIfSlotExistsInSH(recipientId, nextLocalSlotIndex);
        bool txSlotExistsInP2P = checkIfSlotExistsInP2P(selectedNodeId, nextGlobalSlotIndex);
        bool rxSlotExistsInP2P = checkIfSlotExistsInP2P(recipientId, nextGlobalSlotIndex);
        // Check if the recipient has not been assigned in the current slot
        bool isRecipientUniqueForSlot = assignedRecipientsForCurrentSlot.find(recipientId) == assignedRecipientsForCurrentSlot.end();

        EV << "Slot Assignment Details:" << endl
            << "  - Selected Node: " << getHostName(selectedNodeId) << endl
            << "  - Recipient: " << getHostName(recipientId) << endl
            << "  - TX Slot in SH Schedule: " << (txSlotExistsInSH ? "Exists" : "Does Not Exist") << endl
            << "  - RX Slot in SH Schedule: " << (rxSlotExistsInSH ? "Exists" : "Does Not Exist") << endl
            << "  - TX Slot in P2P Schedule: " << (txSlotExistsInP2P ? "Exists" : "Does Not Exist") << endl
            << "  - RX Slot in P2P Schedule: " << (rxSlotExistsInP2P ? "Exists" : "Does Not Exist") << endl
            << "  - Recipient Unique for Slot: " << (isRecipientUniqueForSlot ? "Yes" : "No") << endl;

        // Before the final assignment check, ensure the recipient hasn't been selected for the current slot
        if (!txSlotExistsInSH && !txSlotExistsInP2P && !rxSlotExistsInSH && !rxSlotExistsInP2P && isRecipientUniqueForSlot) {   
            nodeToSlotsMapP2P[selectedNodeId].push_back(nextGlobalSlotIndex); // Assign the selected node to this slot for P2P
            assignedRecipientsForCurrentSlot.insert(recipientId); // Mark this recipient as assigned for the current slot
            // Recalculate number of assigned P2P links for this slot after successful assignment
            numberOfAssignedP2PLinks = countAssignmentsForSlot(nodeToSlotsMapP2P, nextGlobalSlotIndex);

            // Decrement the buffer status for P2P
            if (--bufferStatusP2P[selectedNodeId] <= 0) {
                bufferStatusP2P.erase(selectedNodeId); // Remove from future considerations
            }
            availableNodes.erase(selectedNodeId); // Remove from future considerations in this slot
            availableNodes.erase(recipientId); // Remove from future considerations in this slot
        } else {
            availableNodes.erase(selectedNodeId); // Remove the node if it already has this slot assigned in SH or P2P
            if (rxSlotExistsInSH || rxSlotExistsInP2P) {
                availableNodes.erase(recipientId); // Remove from future considerations in this slot
            }
            // availableNodes.erase(recipientId); // Remove from future considerations in this slot
        }
        recordTransmissionTimeP2P(selectedNodeId, nextSlotStartTime);
    }
    slotToNodesMapP2P = createSlotToNodesMap(nodeToSlotsMapP2P);
    EV << "Assign slots for the point-to-point channel." << endl;
    // printSlotAssignments(slotToNodesMapP2P);
    printNodeSlotAssignments(nodeToSlotsMapP2P);
}

void AbstractLdacsTdmaScheduler::createScheduleSH() {
    // Assuming assignSlotsSH method correctly populates nodeToSlotsMapSH
    assignSlotsSH();

    // Communicate the assigned slots to each corresponding MAC instance
    for (const auto& nodeSlotsPair : nodeToSlotsMapSH) {
        int nodeId = nodeSlotsPair.first;
        const std::vector<int>& assignedSlots = nodeSlotsPair.second;

        // Assuming 'clients' maps node IDs to their corresponding MAC instances
        if (clients.find(nodeId) != clients.end()) {
            clients[nodeId]->setScheduleSH(assignedSlots);
        }
    }
}

void AbstractLdacsTdmaScheduler::createScheduleP2P() {
    assignSlotsP2P(); // Populate nodeToSlotsMapP2P with the new assignments

    for (const auto& nodeSlotsPair : nodeToSlotsMapP2P) {
        int nodeId = nodeSlotsPair.first;
        const std::vector<int>& assignedSlots = nodeSlotsPair.second;

        // Check if this client exists in our client map
        if (clients.find(nodeId) != clients.end()) {
            // Check if only one slot has been assigned
            if (assignedSlots.size() == 1) {
                // Assign the slot to the client
                int assignedSlot = assignedSlots.front();
                clients[nodeId]->setScheduleP2P(assignedSlot);
            } else if (assignedSlots.empty()) {
                // If no slots have been assigned, set to -1 to indicate no slot assignment
                clients[nodeId]->setScheduleP2P(-1);
            } else {
                // More than one slot has been assigned, which is an error under current assumptions
                throw cRuntimeError("Error: Client %d has been assigned more than one slot.", nodeId);
            }
        }
    }
}

void AbstractLdacsTdmaScheduler::updateSlotTimeInfo() {
    currentGlobalSlotIndex = getCurrentGlobalSlotIndex();
    nextGlobalSlotIndex = getNextGlobalSlotIndex();
    currentLocalSlotIndex = getCurrentLocalSlotIndex();
    nextLocalSlotIndex = getNextLocalSlotIndex();
    nextFrameStartGlobalSlotIndex = getNextFrameStartGlobalSlotIndex();
    nextSlotStartTime = getNextSlotStartTime();
    nextFrameStartTime = getNextFrameStartTime();
}

void AbstractLdacsTdmaScheduler::initializeSHAssignment() {
    // Clear previous slot assignments
    nodeToSlotsMapSH.clear();
    slotToNodesMapSH.clear();
    localToGlobalSlotMappingSH.clear();
    for (const auto& node : bufferStatusSH) {
        // Assuming initialization with an empty vector or with predefined values if needed
        nodeToSlotsMapSH[node.first] = std::vector<int>{}; // Initialize with an empty vector
    }

    EV << "SH Buffer Status:" << endl;
    printBufferStatus(bufferStatusSH);
}

void AbstractLdacsTdmaScheduler::initializeP2PAssignment() {
    nodeToSlotsMapP2P.clear();
    for (const auto& node : bufferStatusP2P) {
        nodeToSlotsMapP2P[node.first].clear();
    }
    EV << "P2P Buffer Status:" << endl;
    printBufferStatus(bufferStatusP2P);

    EV_INFO << "Local to Global Slot Mapping:" << endl;
    for (const auto& pair : localToGlobalSlotMappingSH) {
        EV_INFO << "Local Slot ID: " << pair.first << " => Global Slot ID: " << pair.second << endl;
    }
}

int AbstractLdacsTdmaScheduler::getCurrentGlobalSlotIndex() {
    double currentTimeInDbl = simTime().dbl();
    int currentGlobalSlotID = floor(currentTimeInDbl / slotDuration);
    return currentGlobalSlotID;
}

int AbstractLdacsTdmaScheduler::getNextGlobalSlotIndex() {
    int currentGlobalSlotID = getCurrentGlobalSlotIndex();
    int nextGlobalSlotID = currentGlobalSlotID + 1;
    return nextGlobalSlotID;
}

int AbstractLdacsTdmaScheduler::getCurrentLocalSlotIndex() {
    int currentGlobalSlotID = getCurrentGlobalSlotIndex();
    int currentLocalSlotID = findLocalSlotIndex(currentGlobalSlotID);
    return currentLocalSlotID;
}

int AbstractLdacsTdmaScheduler::getNextLocalSlotIndex() {
    int nextGlobalSlotID = getNextGlobalSlotIndex();
    int nextLocalSlotID = findLocalSlotIndex(nextGlobalSlotID);
    return nextLocalSlotID;
}

int AbstractLdacsTdmaScheduler::getCurrentFrameStartGlobalSlotIndex() {
    int currentGlobalSlotID = getCurrentGlobalSlotIndex();
    int currentFrameStartGlobalSlotID = int(currentGlobalSlotID / buildGraphIntervalSlots) * buildGraphIntervalSlots;
    return currentFrameStartGlobalSlotID;
}

int AbstractLdacsTdmaScheduler::getNextFrameStartGlobalSlotIndex() {
    // Calculate the start time of the next frame
    int currentFrameStartGlobalSlotID= getCurrentFrameStartGlobalSlotIndex();
    int nextFrameStartGlobalSlotID = currentFrameStartGlobalSlotID + buildGraphIntervalSlots;
    return nextFrameStartGlobalSlotID;
}

double AbstractLdacsTdmaScheduler::getNextFrameStartTime() {
    int nextFrameStartGlobalSlotID = getNextFrameStartGlobalSlotIndex();
    double nextFrameStart = nextFrameStartGlobalSlotID * slotDuration;
    return nextFrameStart;
}

double AbstractLdacsTdmaScheduler::getNextSlotStartTime() {
    int nextGlobalSlotID = getNextGlobalSlotIndex();
    double nextSlotStart= nextGlobalSlotID * slotDuration;
    return nextSlotStart;
}

std::pair<std::vector<std::vector<int>>, std::unordered_map<int, int>>
AbstractLdacsTdmaScheduler::createAdjacencyMatrixAndNodeMapping() {
    std::unordered_map<int, int> tempMapping;
    std::vector<int> activeNodes;
    int index = 0;

    // Filter nodes with non-empty buffers and prepare temporary mapping
    for (const auto& item : bufferStatusSH) {
        if (item.second > 0) { // Check for non-empty buffer
            tempMapping[item.first] = index;
            activeNodes.push_back(item.first);
            ++index;
        }
    }

    int activeCount = activeNodes.size();
    std::vector<std::vector<int>> adjacencyMatrix(activeCount, std::vector<int>(activeCount, 0));

    // Fill adjacency matrix
    for (int i = 0; i < activeCount; ++i) {
        auto mobilityI = mobilityModules[activeNodes[i]];
        for (int j = i + 1; j < activeCount; ++j) { // Start from i+1 to avoid duplicate calculations
            auto mobilityJ = mobilityModules[activeNodes[j]];
            double distance = mobilityI->getCurrentPosition().distance(mobilityJ->getCurrentPosition());

            // Check if within communication range (Use this->communicationRange directly)
            if (distance <= this->communicationRange) {
                adjacencyMatrix[i][j] = adjacencyMatrix[j][i] = 1; // Symmetric matrix, use integers 1 for true, 0 for false
            }
        }
    }

    return {adjacencyMatrix, tempMapping};
}

void AbstractLdacsTdmaScheduler::buildGraph() {
    // Implementation of your graph building or updating logic
    EV << "Building or updating the graph at " << simTime() << endl;
    // Retrieve adjacency matrix and node mapping
    auto result = createAdjacencyMatrixAndNodeMapping();
    adjacencyMatrix = std::move(result.first); // Store the adjacency matrix in the class member variable
    nodeMapping = std::move(result.second); // Store the node mapping in the class member variable

    // Print the adjacency matrix
    EV << "Adjacency Matrix:" << endl;
    for (int i = 0; i < adjacencyMatrix.size(); i++) {
        for (int j = 0; j < adjacencyMatrix[i].size(); j++) {
            EV << adjacencyMatrix[i][j] << " ";
        }
        EV << endl;
    }

    // Print the node mapping
    EV << "Node Mapping:" << endl;
    for (const auto& pair : nodeMapping) {
        EV << "Node ID " << pair.first << " maps to Index " << pair.second << endl;
    }
}

std::vector<int> AbstractLdacsTdmaScheduler::findNodesWithinOneAndTwoHops(int NodeID) {
    std::unordered_set<int> resultNodes; // Use a set to avoid duplicates
    std::vector<int> hopsOneAndTwo;

    // Check if the NodeID exists in nodeMapping
    if (nodeMapping.find(NodeID) == nodeMapping.end()) {
        return {}; // Return an empty vector if the node ID doesn't exist
    }

    int nodeGraphID = nodeMapping[NodeID]; // Get the index in the adjacency matrix for the given node ID

    // First, add all 1-hop neighbors
    for (int i = 0; i < adjacencyMatrix[nodeGraphID].size(); ++i) {
        if (adjacencyMatrix[nodeGraphID][i] == 1) { // Direct connection exists
            resultNodes.insert(i); // Add to results as index; convert to ID later
            // Then, find 2-hop neighbors by exploring the neighbors of i
            for (int j = 0; j < adjacencyMatrix[i].size(); ++j) {
                if (adjacencyMatrix[i][j] == 1 && j != nodeGraphID) { // Ensure not to add the original node
                    resultNodes.insert(j); // Add to results as index; convert to ID later
                }
            }
        }
    }

    // Convert indices back to node IDs
    for (const auto& idx : resultNodes) {
        // Assuming a reverse mapping exists or you can find IDs by index
        for (const auto& mapping : nodeMapping) {
            if (mapping.second == idx) {
                hopsOneAndTwo.push_back(mapping.first);
                break; // Found the ID for this index, move to next
            }
        }
    }

    return hopsOneAndTwo;
}

std::string AbstractLdacsTdmaScheduler::getHostName(int nodeId) {
    auto clientIt = clients.find(nodeId);
    if (clientIt != clients.end()) {
        cModule* macModule = clientIt->second;
        cModule* wlanModule = macModule->getParentModule();
        cModule* hostModule = wlanModule->getParentModule();
        return hostModule->getFullName();
    }
    return "Unknown"; // Return a default or error name if not found
}

SlotToNodesMap AbstractLdacsTdmaScheduler::createSlotToNodesMap(const NodeToSlotsMap& nodeToSlotsMap) {
    // Initialize slotToNodesMap with empty vectors for all slots
    SlotToNodesMap slotToNodesMap;
    for (int slot = 0; slot < buildGraphIntervalSlots; ++slot) {
        slotToNodesMap[slot] = std::vector<int>(); // Ensure every slot is represented
    }

    // Populate slotToNodesMap with node assignments from nodeToSlotsMap
    for (const auto& assignment : nodeToSlotsMap) {
        for (int slot : assignment.second) {
            slotToNodesMap[slot].push_back(assignment.first);
        }
    }

    return slotToNodesMap; // Return the populated map
}

void AbstractLdacsTdmaScheduler::printSlotAssignments(const SlotToNodesMap& slotToNodesMap) {
    auto slotToNodesMapNew = slotToNodesMap;

    EV << "Slot        |   NodeIds" << endl;
    EV << "------------+--------------" << endl;
    
    // Print the slot assignments in the desired format
    for (int slot = 0; slot < buildGraphIntervalSlots; ++slot) {
        EV << "       " << slot << "    |   ";
        if (!slotToNodesMapNew[slot].empty()) {
            for (auto nodeId : slotToNodesMapNew[slot]) {
                EV << getHostName(nodeId) << " ";
            }
        } else {
            EV << "None"; // Or simply leave blank if no nodes are assigned to this slot
        }
        EV << endl;
    }
}

void AbstractLdacsTdmaScheduler::printNodeSlotAssignments(const NodeToSlotsMap& nodeToSlotsMap) {
    EV << std::left << std::setw(20) << "NodeID" << "|     Global Slot ID" << endl;
    EV << "--------------------+-------------------" << endl;
    
    // Iterate through each node and their assigned slots
    for (const auto& nodeSlotsPair : nodeToSlotsMap) {
        std::string nodeName = getHostName(nodeSlotsPair.first); // Assuming getHostName converts nodeId to a host name string
        
        // Print the node name with consistent spacing
        EV << std::left << std::setw(20) << nodeName << "|   ";
        
        // Check if the node has slots assigned
        if (!nodeSlotsPair.second.empty()) {
            std::stringstream slotsStream;
            for (auto slot : nodeSlotsPair.second) {
                slotsStream << slot << " ";
            }
            // Print the slots with consistent spacing
            EV << std::left << std::setw(15) << slotsStream.str();
        } else {
            EV << std::left << std::setw(15) << "None"; // Or simply leave blank if no slots are assigned to this node
        }
        EV << endl;
    }
}

void AbstractLdacsTdmaScheduler::printBufferStatus(const map<int, int>& buffer) {
    EV << "       " << std::left << std::setw(20) << "Node" << "|   Buffer Status" << endl;
    EV << "---------------------------+----------------" << endl;
    for (const auto& node : buffer) {
        if (node.second >= 0) {
            std::string nodeName = getHostName(node.first); // Assuming getHostName returns a string
            // Adjust the width according to your needs
            EV << "       " << std::left << std::setw(20) << nodeName << "|   ";
            EV << std::left << std::setw(15) << node.second << endl;
        }
    }
}

int AbstractLdacsTdmaScheduler::findLocalSlotIndex(int currentGlobalSlotID) {
    int localSlotID = -1; // Initialize with an invalid value
    for (const auto& localGlobalPair : localToGlobalSlotMappingSH) {
        if (localGlobalPair.second == currentGlobalSlotID) {
            localSlotID = localGlobalPair.first;
            break; // Found the corresponding local slot index
        }
    }
    return localSlotID;
}

// Populates the set of available nodes based on their eligibility and buffer status.
std::unordered_set<int> AbstractLdacsTdmaScheduler::populateAvailableNodesSH(double slotStart) {
    std::unordered_set<int> availableNodes;
    // Populate availableNodes with nodes that have a positive buffer status for P2P
    // Determine available nodes based on the copied nodeMapping and buffer status
    for (const auto& node : nodeMapping) {
        if (bufferStatusSH[node.first] > 0) {
            // Check if the node is eligible for reassignment based on the last assignment time
            auto lastAssignedIt = lastAssignedSH.find(node.first);
            bool isEligibleForReassignment = true;
            if (lastAssignedIt != lastAssignedSH.end()) {
                // Calculate elapsed time since last assignment for this node
                simtime_t elapsedTimeSinceLastAssignment = slotStart - lastAssignedIt->second;
                if (elapsedTimeSinceLastAssignment < minReassignmentDurationSH) {
                    isEligibleForReassignment = false;
                }
            }
            if (isEligibleForReassignment) {
                availableNodes.insert(node.first);
            }
        }
    }
    return availableNodes;
}

// Populates the set of available nodes based on their eligibility and buffer status.
std::unordered_set<int> AbstractLdacsTdmaScheduler::populateAvailableNodesP2P(double slotStart) {
    std::unordered_set<int> availableNodes;
    // Populate availableNodes with nodes that have a positive buffer status for P2P
    for (const auto& node : bufferStatusP2P) {
        if (bufferStatusP2P[node.first] > 0) {
            auto lastAssignedIt = lastAssignedP2P.find(node.first); // Or lastAssignedP2P for P2P
            bool isEligibleForReassignment = true;
            if (lastAssignedIt != lastAssignedP2P.end()) {
                // Calculate elapsed time since last assignment for this node
                simtime_t elapsedTimeSinceLastAssignment = slotStart - lastAssignedIt->second;
                bool checkoutput = elapsedTimeSinceLastAssignment < minReassignmentDurationP2P;
                if (elapsedTimeSinceLastAssignment < minReassignmentDurationP2P) { // Or minReassignmentDurationP2P for P2P
                    isEligibleForReassignment = false;
                }
            }
            if (isEligibleForReassignment) {
                availableNodes.insert(node.first);
            }
        }
    }
    return availableNodes;
}

bool AbstractLdacsTdmaScheduler::checkIfSlotExistsInSH(int nodeId, int localSlotIndex) {
    // Check if nodeId or recipientId exists in nodeToSlotsMapSH and if slot is in its vector
    bool slotExistsInSH = false;
    if (nodeToSlotsMapSH.find(nodeId) != nodeToSlotsMapSH.end()) {
        slotExistsInSH = std::find(nodeToSlotsMapSH[nodeId].begin(), 
                                nodeToSlotsMapSH[nodeId].end(), localSlotIndex) != nodeToSlotsMapSH[nodeId].end();
    }
    return slotExistsInSH;
}

bool AbstractLdacsTdmaScheduler::checkIfSlotExistsInP2P(int nodeId, int globalSlotIndex) {
    // Check if nodeId or recipientId exists in nodeToSlotsMapSH and if slot is in its vector
    bool slotExistsInP2P = false;
    if (nodeToSlotsMapP2P.find(nodeId) != nodeToSlotsMapP2P.end()) {
            slotExistsInP2P = std::find(nodeToSlotsMapP2P[nodeId].begin(), 
                                        nodeToSlotsMapP2P[nodeId].end(), globalSlotIndex) != nodeToSlotsMapP2P[nodeId].end();
    }
    return slotExistsInP2P;
}

int AbstractLdacsTdmaScheduler::findNodeIdByMac(MacAddress macAddress) {
    int nodeId = -1;
    // Find the recipient's nodeId using its MAC address
    for (const auto& client : clientsMacAddress) {
        if (client.second == macAddress) {
            // EV_INFO << "Mac of recepient is: " << recipientMac << endl;
            nodeId = client.first;
            break;
        }
    }
    return nodeId;
} 

int AbstractLdacsTdmaScheduler::selectRandomNode(const std::unordered_set<int>& availableNodes) {
    if (availableNodes.empty()) {
        throw std::runtime_error("No available nodes to select.");
    }

    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<> dist(0, availableNodes.size() - 1);

    auto it = availableNodes.begin();
    std::advance(it, dist(gen)); // Advance iterator it by the random distance
    return *it; // Dereference iterator to get the selected node ID
}



int AbstractLdacsTdmaScheduler::countAssignmentsForSlot(const std::unordered_map<int, std::vector<int>>& map, int slotID) {
    int count = 0;
    for (const auto& pair : map) {
        if (std::find(pair.second.begin(), pair.second.end(), slotID) != pair.second.end()) {
            ++count;
        }
    }
    return count;
}