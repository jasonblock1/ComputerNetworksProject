/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"

typedef nx_struct neighbor {
    nx_uint16_t Node;
    nx_uint16_t Age;
} neighbor;

typedef nx_struct lspLink{
	nx_uint16_t neighbor;
	nx_uint8_t cost;
	nx_uint8_t src;
}lspLink;

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;
	uses interface List<pack> as nodesVisited;
   	uses interface List<pack> as ListOfNeighbors;
    	//uses interface Timer<TMilli> as NeighborTimer;
	uses interface List<lspLink> as lspLinkList;
}

implementation{
   pack sendPackage;
	uint16_t sequence = 1;
	uint32_t start, offset;
    	pack currentNeighbor;
	char* neighborPayload = "Neighbor Discovery";
	lspLink lsp;
	
	bool checkNeighbors(pack *Package);

	bool pkgValid(pack *Package);

   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
	
	//uint sequence
   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      //dbg(GENERAL_CHANNEL, "Packet Received\n");
      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
         //dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);
	if(!pkgValid(myMsg)) {
		//dbg(FLOODING_CHANNEL, "Package removed\n\n");
		return msg;
	} 
	
	if(myMsg->protocol == PROTOCOL_PING) {
		if(TOS_NODE_ID == myMsg->dest) {
			dbg(FLOODING_CHANNEL, "Success!: Package from Node: %d, at destination Node: %d, Package Payload: %s\n\n", myMsg->src, myMsg->dest, myMsg->payload);
			return msg;
		}else {
			dbg(FLOODING_CHANNEL, "Package Received from %d at Node: %d\n\n", myMsg->src, TOS_NODE_ID);
			makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL - 1, myMsg->protocol = PROTOCOL_PING, myMsg->seq, myMsg->payload, sizeof(myMsg->payload));
			call nodesVisited.pushback(*myMsg);
			call Sender.send(sendPackage, AM_BROADCAST_ADDR);	
		}
		
	}else if(myMsg->protocol == PROTOCOL_PINGREPLY) {	//handle neighbor discovery package
		if(checkNeighbors(myMsg)) {
			dbg(NEIGHBOR_CHANNEL, "New neighbor found!\n");
			makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 1, myMsg->protocol = PROTOCOL_PINGREPLY, myMsg->seq, (uint8_t*) neighborPayload, PACKET_MAX_PAYLOAD_SIZE);
			call Sender.send(sendPackage, AM_BROADCAST_ADDR);
		}
	}else if(myMsg->protocol == PROTOCOL_LINKSTATE) {
		uint16_t i,j = 0;
        	uint16_t k = 0;
        	bool validData = TRUE;
                for(i = 0; i < myMsg->seq; i++) {
			for(j = 0; j < call lspLinkList.size(); j++) {
	                  	lspLink lspacket = call lspLinkList.get(j);
	                  	
				if(lspacket.src == myMsg->src && lspacket.neighbor==myMsg->payload[i]) {
	                    	validData = FALSE;
	                  	}
	                }
		}
		
		if(validData) {
			for(k = 0; k < myMsg->seq; k++) {
                  		lsp.neighbor = myMsg->payload[k];
                  		lsp.cost = 1;
                  		lsp.src = myMsg->src;
                  		call lspLinkList.pushback(lsp);
                  		//dbg(ROUTING_CHANNEL,"$$$Neighbor: %d\n",lspL.neighbor);
                	}
               		makePack(&sendPackage, myMsg->src, AM_BROADCAST_ADDR, myMsg->TTL-1 , PROTOCOL_LINKSTATE, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                //Check TOS_NODE_ID and destination
                	call Sender.send(sendPackage,AM_BROADCAST_ADDR);
		}else{
                //dbg(ROUTING_CHANNEL,"LSP already exists for %d\n",TOS_NODE_ID);
		}
	}	
		return msg;
	
      }
      dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
      return msg;
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      if(call routingTable.contains(destination))
        {
          makePack(&sendPackage, TOS_NODE_ID, destination, MAX_TTL, PROTOCOL_PING, sequence, payload, PACKET_MAX_PAYLOAD_SIZE);
          dbg(NEIGHBOR_CHANNEL, "To get to:%d, send through:%d\n", destination, call routingTable.get(destination));
          call Sender.send(sendPackage, call routingTable.get(destination));
        }
        else{
          makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 1, PROTOCOL_PINGREPLY, sequence, (uint8_t*) neighborPayload, PACKET_MAX_PAYLOAD_SIZE);
          dbg(NEIGHBOR_CHANNEL, "No Routing Table for:%d so flooding\n", TOS_NODE_ID);
          call Sender.send(sendPackage, AM_BROADCAST_ADDR);
	
	sequence++;
   }

	bool pkgValid(pack* myMsg) {
        	uint16_t list = call nodesVisited.size();
        	uint16_t i = 0;
        
        
       		if (myMsg->TTL == 0) {
            		return FALSE;
        	} else if (list == 0) {
           		return TRUE;
        	} else {
            		for (i = 0; i < list; i++) {
                
                		pack currentPack;
                		currentPack = call nodesVisited.get(i);
                
                		if (currentPack.dest == myMsg->dest && currentPack.seq == myMsg->seq) {
                    		return FALSE;
                		}
           		}
            		return TRUE;
        	}
    	}

	bool checkNeighbors(pack* myMsg) {
		uint16_t list = call ListOfNeighbors.size();
		uint16_t i = 0;
		
		//Neighbor->Node = myMsg->src;
		//Neighbor->Age = 1;

		if(list == 0) {
			return TRUE;
		}else {
			for (i=0; i < list; i++) {
				currentNeighbor = call ListOfNeighbors.get(i);

				if(currentNeighbor.src == myMsg->src) {
					return FALSE;
				}
			}

			return TRUE;
		}
	}

   event void CommandHandler.printNeighbors(){
		uint16_t list = call ListOfNeighbors.size();
		uint16_t i = 0;

		if(list > 0) {
			for(i=0; i < list; i++) {
				currentNeighbor = call ListOfNeighbors.get(i);
		
				dbg(NEIGHBOR_CHANNEL, "NEIGHBORS of node %d is :%d\n",TOS_NODE_ID,currentNeighbor.src);
			}
		}else {
				dbg(NEIGHBOR_CHANNEL, "Node %d has no neighbors\n",TOS_NODE_ID);	
		}	
	}	
	

   event void CommandHandler.printRouteTable(){}

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }
}
