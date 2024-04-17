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
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package ldacs_abstract_tdma.scheduler;

//
// TODO auto-generated type
//
module AbstractLdacsTdmaScheduler
{
    parameters:
        bool monitorSchedule = default(false);
        double slotDuration @unit(s) = default(24ms);
        int frameLength = default(10);
        double communicationRange @unit(m); // the range between nodes
        int buildGraphIntervalSlots = default(10); // Number of slots after which the graph should be rebuilt
        int minReassignmentSlotsSH = default(0); // the minimum time before a node gets assigned again in slots of the SH channel
        int minReassignmentSlotsP2P = default(0); // the minimum time before a node gets assigned again in slots of the P2P channel
        int maxP2PLinks = default(50); // the maxiximum number of usabel P2P links in a specific location

    	@class(AbstractLdacsTdmaScheduler);
    	
        @signal[schedule](type=long);
        @signal[utilization](type=double);
        @statistic[schedule](title="schedule"; record=vector,histogram; interpolationmode=none);
        @statistic[utilization](title="utilization"; record=vector,histogram,timeavg; interpolationmode=none);
        ///////////////////////////////////
        /// (record link access delay)
        @signal[nodeId](type=long); // Declare the signal in NED file
        @statistic[nodeId](record=vector);
}
