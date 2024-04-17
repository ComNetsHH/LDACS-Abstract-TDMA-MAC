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

import inet.linklayer.acking.AckingMac;
import inet.common.LayeredProtocolBase;
import inet.queueing.contract.IPacketQueue;

package ldacs_abstract_tdma.mac;

//
// TODO auto-generated type
//
module AbstractLdacsTdmaMac extends AckingMac
{
    parameters:
        string scheduler = "^.^.^.scheduler";
        double slotDuration @unit(s) = default(2ms);
        int frameLength = default(10);
        int numRetries = default(3);
        int buildGraphIntervalSlots = default(10); // Number of slots after which the graph should be rebuilt
        int maxP2PLinks = default(50); // the maxiximum number of usabel P2P links in a specific location
        
        @signal[macDelaySH](type="simtime_t");
        @statistic[macDelaySH](source="macDelaySH"; record=vector, histogram, mean, max, min);
        @signal[macDelayP2P](type="simtime_t");
        @statistic[macDelayP2P](source="macDelayP2P"; record=vector, histogram, mean, max, min);
        
        @class(AbstractLdacsTdmaMac);  
    submodules:
        queueP2P: <default("DropTailQueue")> like IPacketQueue {
            parameters:
                @display("p=150,100;q=l2queue");
        }

}
