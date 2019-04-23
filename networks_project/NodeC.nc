/**
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */

#include <Timer.h>
#include "includes/CommandMsg.h"
#include "includes/packet.h"

configuration NodeC{
}
implementation {
    components MainC;
    components Node;
    components new AMReceiverC(AM_PACK) as GeneralReceive;

    Node -> MainC.Boot;

    Node.Receive -> GeneralReceive;

    components ActiveMessageC;
    Node.AMControl -> ActiveMessageC;

    components new SimpleSendC(AM_PACK);
    Node.Sender -> SimpleSendC;


    components CommandHandlerC;
    Node.CommandHandler -> CommandHandlerC;

    components 	RandomC as Random;
    Node.Random -> Random;

    components new TimerMilliC() as NeighborDiscoveryTimerC;
    Node.NeighborDiscoveryTimer -> NeighborDiscoveryTimerC;

    components new TimerMilliC() as lsrTimerC;
    Node.lsrTimer -> lsrTimerC;

    components new TimerMilliC() as dijkstraTimerC;
    Node.dijkstraTimer -> dijkstraTimerC;

    components new ListC(pack, 20) as nodesVisitedC;
    Node.nodesVisited -> nodesVisitedC;

    components new ListC(pack, 20) as ListOfNeighborsC;
    Node.ListOfNeighbors -> ListOfNeighborsC;


    components new HashmapC(int, 300) as routingTableC;
    Node.routingTable -> routingTableC;

    components new ListC(lspLink, 64) as lspLinkListC;
    Node.lspLinkList -> lspLinkListC;

    components TransportC;
    Node.Transport -> TransportC; 
 
    components RoutingTableC;
    Node.RoutingTable -> RoutingTableC;

    


    //components LinkStateC;//Change before submission*****
    //Node.LinkState -> LinkStateC;
    //Node.lspLinkList -> lspLinkC;
    //Node.routingTable ->HashmapC;
    //LinkStateC.neighborListC-> neighborListC;
    //LinkStateC.HashmapC -> HashmapC;


}
