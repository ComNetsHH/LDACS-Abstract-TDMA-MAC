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

import ldacs_abstract_tdma.mac.AbstractLdacsTdmaMac;
import ldacs_abstract_tdma.rlc.Rlc;
import ldacs_abstract_tdma.rlc.IRlc;
import inet.queueing.contract.IPacketQueue;
import inet.linklayer.contract.IWirelessInterface;
import inet.networklayer.common.InterfaceEntry;
import inet.physicallayer.contract.packetlevel.IRadio;
import ldacs_abstract_radio.LdacsAbstractRadio;


package ldacs_abstract_tdma;

//
// TODO auto-generated type
//
network TdmaInterface extends InterfaceEntry like IWirelessInterface
{
    parameters:
        @class(::inet::InterfaceEntry);
        @display("i=block/ifcard");
        string interfaceTableModule;
        string energySourceModule = default("");
        double bitrate @unit(bps);
        *.interfaceTableModule = default(absPath(interfaceTableModule));
        *.energySourceModule = default(absPath(energySourceModule));
        **.bitrate = bitrate;
    gates:
        input upperLayerIn;
        output upperLayerOut;
        input radioIn @labels(Signal);
    submodules:
        mac: AbstractLdacsTdmaMac {
            parameters:
                @display("p=100,200");
        }
        radio: <default("LdacsAbstractRadio")> like IRadio {
            parameters:
                @display("p=100,300");
        }
    connections:
        upperLayerIn --> mac.upperLayerIn;
        mac.upperLayerOut --> upperLayerOut;
        mac.lowerLayerOut --> radio.upperLayerIn;
        radio.upperLayerOut --> mac.lowerLayerIn;
        radioIn --> { @display("m=s"); } --> radio.radioIn;
}


