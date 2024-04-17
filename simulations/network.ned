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
import ldacs_abstract_tdma.scheduler.AbstractLdacsTdmaScheduler;
import ldacs_abstract_radio.LdacsAbstractRadio;
import udpbroadcastapp.UdpBroadcastApp;
import inet.node.inet.AdhocHost;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.unitdisk.UnitDiskRadioMedium;
import inet.visualizer.contract.IIntegratedVisualizer;
import inet.node.inet.AdhocHost;
import inet.node.inet.ManetRouter;


package ldacs_abstract_tdma.simulations;

//
// TODO auto-generated type
//
network Network
{
    parameters:
        // int numNodes = default(4);
        @display("bgb=400,200;bgl=2");
    submodules:
    	// nodes[numNodes]: AdhocHost {}
    	scheduler: AbstractLdacsTdmaScheduler {
            parameters:
                @display("p=100,400;is=s");
        }
    	//visualizer: <default("IntegratedCanvasVisualizer")> like IIntegratedVisualizer if hasVisualizer() {
       	//	parameters:
        //        @display("p=100,300;is=s");
        //}
        configurator: Ipv4NetworkConfigurator {
            parameters:
                @display("p=100,100;is=s");
        }
        radioMedium: UnitDiskRadioMedium {
            parameters:
                @display("p=100,200;is=s");
        }
        node1: ManetRouter {
            parameters:
                @display("p=50,100");
        }
        node2: ManetRouter {
            parameters:
                @display("p=100,100");
        }
        node3: ManetRouter {
            parameters:
                @display("p=150,100");
        }
        node4: ManetRouter {
            parameters:
                @display("p=200,100");
        }
        node5: ManetRouter {
            parameters:
                @display("p=250,100");
        }
        node6: ManetRouter {
            parameters:
                @display("p=300,100");
        }
        receiver: ManetRouter {
            parameters:
                @display("p=350,100");
        }
}
