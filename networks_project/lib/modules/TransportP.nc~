#include "../../includes/socket.h"
#include "../../includes/packet.h"
#include "../../includes/TCPPacket.h"
#include "../../includes/protocol.h"
#include <Timer.h>

module TransportP{
   provides interface Transport;

   uses interface Random as Random;
   uses interface SimpleSend as Sender;
   uses interface RoutingTable;
   uses interface List<socket_t> as SocketList;
   uses interface List<pack> as SendPacketQueue;
   uses interface List<pack> as RcvdPacketQueue;
   uses interface Timer<TMilli> as ServerReadTimer;
   uses interface Timer<TMilli> as ClientWriteTimer;
   uses interface Timer<TMilli> as SendTimer;
   uses interface Timer<TMilli> as TimeWait;
   uses interface Timer<TMilli> as TimeoutTimer;
}

implementation{
   uint8_t socketIndex = 1;
   uint8_t maxSockets = 10;
   socket_t activeSockets[10];
   socket_t getSocket(uint8_t destPort, uint8_t srcPort);
   void connectFinish(socket_t fd);

	void connectFinish(socket_t fd) {
		pack myMsg;
		tcp_pack* myTCPPack;
		socket_t mySocket = fd;
		uint8_t i = 0;
		call SendPacketQueue.popfront();
		call TimeoutTimer.stop();
		
		//Start timers
		activeSockets[mySocket.fd] = mySocket;
		dbg(TRANSPORT_CHANNEL, "STARTING TRANSMISSION\n");
		call ClientWriteTimer.startPeriodic(1000);
		call SendTimer.startPeriodic(150000);
				
	}

	socket_t getSocket(uint8_t destPort, uint8_t srcPort){
		socket_t mySocket;
		uint32_t i = 0;
		//Returns current socket
		
		for (i = 0; i < maxSockets; i++){
			mySocket = activeSockets[i];
			if(mySocket.dest.port == srcPort && mySocket.src.port == destPort){
				return mySocket;
			}
		}
		
	}
   	
 
  command socket_fd_t Transport.socket(){
	//Assign file descriptors to sockets   	
	if(socketIndex <= 10) {
		uint8_t fd = socketIndex;
		socketIndex++;
		return (socket_fd_t)fd;
	}else {
		return (socket_fd_t)NULL;
	}
  }
	
   
  

  command error_t Transport.bind(socket_fd_t fd, socket_addr_t *addr){
  }

  command socket_t Transport.accept(socket_t server){
	
	uint16_t i = 0;
	socket_t mySocket;

	for(i = 0; i < maxSockets; i++) {
		mySocket = activeSockets[i];
		if(mySocket.dest.port == server.src.port && server.state == LISTEN) {
			dbg(ROUTING_CHANNEL, "CLIENT FOUND\n");
			server.dest.addr = mySocket.src.addr;
			server.dest.port = mySocket.src.port;
			return server;
		}
	}

  }

  command uint16_t Transport.write(socket_t fd, uint8_t *buff, uint16_t bufflen){
	uint16_t i = fd.lastWritten;
	uint16_t bytesWritten = 0;
	pack myMsg;
	tcp_pack* myTCPPack;
	//Initialize TCP header info
	myTCPPack = (tcp_pack*)(myMsg.payload);
	myTCPPack->flag = DATA_FLAG;
	myTCPPack->srcPort = fd.src.port;
	myTCPPack->destPort = fd.dest.port;
	//Client: Package buffered data in frames and add frames to a queue
	for(i = fd.lastWritten; i < fd.transfer && i < bufflen; i++) {
		if(call SendPacketQueue.size() < 100) {
			myTCPPack->payload[0] = buff[i];
			myTCPPack->seq = i + 1;
			dbg(TRANSPORT_CHANNEL, "%hu\n", myTCPPack->seq);
			call Transport.makePack(&myMsg, TOS_NODE_ID, fd.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
		
			call SendPacketQueue.pushback(myMsg);
			bytesWritten++;
		}else {
			return bytesWritten;
		}
	}

	return bytesWritten;
  

  }
  
  command uint16_t Transport.read(socket_t fd, uint8_t *buff, uint16_t bufflen){
  	uint16_t i = fd.lastRead;
	uint16_t bytesRead = 0;
	pack myMsg;
	tcp_pack* myTCPPack;
	//Server: Read data from frames and add to receive buffer
	myTCPPack = (tcp_pack*)(myMsg.payload);
	dbg(TRANSPORT_CHANNEL, "READING DATA...\n");

	buff[myTCPPack->seq] = myTCPPack->payload[0];
	myTCPPack->ACK = myTCPPack->seq;
	myTCPPack->flag = DATA_ACK_FLAG;
	myTCPPack->srcPort = fd.src.port;
	myTCPPack->destPort = fd.dest.port;
	dbg(TRANSPORT_CHANNEL, "%hhu\n", myTCPPack->ACK);
	call Transport.makePack(&myMsg, TOS_NODE_ID, fd.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
	//Send ACK	
	call Sender.send(myMsg, (call RoutingTable.get(fd.dest.addr)));
	bytesRead++;
		

	
	return bytesRead;
	
     
  }
  
  command error_t Transport.connect(socket_t fd){
	socket_t mySocket = fd;
	pack myMsg;
	tcp_pack* myTCPpack;
	uint8_t i = 0;
	//Client: Initialize and send SYN packets
	myTCPpack = (tcp_pack*)(myMsg.payload);
	myTCPpack->srcPort = mySocket.src.port;
	myTCPpack->destPort = mySocket.dest.port;
	myTCPpack->flag = SYN_FLAG;
	myTCPpack->seq = 1;

	call Transport.makePack(&myMsg, TOS_NODE_ID, mySocket.dest.addr, 20, PROTOCOL_TCP, 27, myTCPpack, PACKET_MAX_PAYLOAD_SIZE);
	mySocket.state = SYN_SENT;

	dbg(TRANSPORT_CHANNEL, "CLIENT CONNECTING...%d\n",mySocket.dest.addr);
	call SendPacketQueue.pushback(myMsg);
	call SendTimer.startOneShot(10);
	
  }
   command error_t Transport.close(socket_t fd){
        pack myMsg;
	tcp_pack* myTCPPack;
	//Close connection when all data has been read and acknowledged
	myTCPPack = (tcp_pack*)(myMsg.payload);
	myTCPPack->srcPort = fd.dest.port;
	myTCPPack->destPort = fd.src.port;
	if(fd.state == ESTABLISHED) {
		myTCPPack->flag = FIN_FLAG;
		//change socket state to FIN_WAIT
		fd.state = FIN_WAIT;
		activeSockets[fd.fd] = fd;

		call Transport.makePack(&myMsg, TOS_NODE_ID, fd.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
		//Send FIN packet
		dbg(TRANSPORT_CHANNEL, "FIN WAIT...\n");
		call SendPacketQueue.pushback(myMsg);
		call SendTimer.startOneShot(10);
		
	}else if(fd.state == FIN_WAIT) {
		//Send FIN_ACK packet
		myTCPPack->flag = FIN_ACK_FLAG;
		//Change socket state to TIME_WAIT
		fd.state = TIME_WAIT;		
		activeSockets[fd.fd] = fd;
		call Transport.makePack(&myMsg, TOS_NODE_ID, fd.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
		call SendPacketQueue.pushback(myMsg);
		call SendTimer.startOneShot(10);
		
	
		dbg(TRANSPORT_CHANNEL, "TIME WAIT...\n");

		call TimeWait.startOneShot(5000);

	}else if(fd.state == TIME_WAIT) {
		//Change socket state to closed
		uint8_t i = 1;
		call SendPacketQueue.popfront();
		call TimeoutTimer.stop();
		dbg(TRANSPORT_CHANNEL, "CONNECTION CLOSED\n");
		//Server: Print read data
		while(fd.rcvdBuff[i] != 0) {
			dbg(TRANSPORT_CHANNEL, "DATA: %hhu \n", fd.rcvdBuff[i]);
			i++;
		}
		fd.state = CLOSED;
		activeSockets[fd.fd] = fd;
	}
   }
  command error_t Transport.receive(pack* msg){

		uint8_t srcPort = 0;
		uint8_t destPort = 0;
		uint8_t seq = 1;
		uint8_t lastAck = 0;
		uint8_t flag = 0;
		uint16_t bufflen = TCP_PACKET_MAX_PAYLOAD_SIZE;
		uint16_t i = 0;
		uint16_t j = 0;
		uint32_t key = 0;
		uint8_t SeqNumToAck = 0; 
		socket_t mySocket;
		socket_t connectedSocket; 
		tcp_pack* myMsg = (tcp_pack *)(msg->payload);


		pack myNewMsg;
		tcp_pack* myTCPPack;
		
		
		srcPort = myMsg->srcPort;
		destPort = myMsg->destPort;
		seq = myMsg->seq;
		lastAck = myMsg->ACK;
		flag = myMsg->flag;
		
		if(flag == SYN_FLAG || flag == SYN_ACK_FLAG || flag == ACK_FLAG){

			if(flag == SYN_FLAG){
				dbg(TRANSPORT_CHANNEL, "Received SYN! \n");

				for(i = 0; i < maxSockets; i++) {
				mySocket = activeSockets[i];
					if(mySocket.src.port == destPort) {
						dbg(ROUTING_CHANNEL, "ATTEMPTING CONNECTION WITH CLIENT\n");
						break;
					}else if(i == maxSockets - 1) {
						dbg(TRANSPORT_CHANNEL, "CONNECTION FAILED, NO SOCKET W/ PORT %d\n", destPort);
						return FAIL;
					}
				
				}
				
					
				mySocket.state = SYN_RCVD;
				mySocket.dest.port = srcPort;
				mySocket.dest.addr = msg->src;
						
				activeSockets[mySocket.fd] = mySocket;
				myTCPPack = (tcp_pack *)(myNewMsg.payload);
				myTCPPack->destPort = mySocket.dest.port;
				myTCPPack->srcPort = mySocket.src.port;
				myTCPPack->seq = 1;
				myTCPPack->ACK = seq + 1;
				myTCPPack->flag = SYN_ACK_FLAG;
				dbg(TRANSPORT_CHANNEL, "Sending SYN ACK! \n");
				mySocket.state = 4;
				activeSockets[mySocket.fd] = mySocket;
				call Transport.makePack(&myNewMsg, TOS_NODE_ID, mySocket.dest.addr, 20, 4, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
				call Sender.send(myNewMsg, (call RoutingTable.get(mySocket.dest.addr)));	
			}
			
			else if(flag == SYN_ACK_FLAG){
				dbg(TRANSPORT_CHANNEL, "Received SYN ACK! \n");
				mySocket = getSocket(destPort, srcPort);
				call SendPacketQueue.popfront();
				call TimeoutTimer.stop();
				mySocket.state = ESTABLISHED;
				
				activeSockets[mySocket.fd] = mySocket;
				myTCPPack = (tcp_pack*)(myNewMsg.payload);
				myTCPPack->destPort = mySocket.dest.port;
				myTCPPack->srcPort = mySocket.src.port;
				myTCPPack->seq = 1;
				myTCPPack->ACK = seq + 1;
				myTCPPack->flag = ACK_FLAG;
				dbg(TRANSPORT_CHANNEL, "SENDING ACK \n");
				call Transport.makePack(&myNewMsg, TOS_NODE_ID, mySocket.dest.addr, 20, 4, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
				call Sender.send(myNewMsg, (call RoutingTable.get(mySocket.dest.addr)));
				dbg(TRANSPORT_CHANNEL, "CONNECTION ESTABLISHED! \n");
				connectFinish(mySocket);
			}

			else if(flag == ACK_FLAG){
				dbg(TRANSPORT_CHANNEL, "GOT ACK \n");
				mySocket = getSocket(destPort, srcPort);
				mySocket.state = ESTABLISHED;
					
				activeSockets[mySocket.fd] = mySocket;
				
			}
		}else if(flag == DATA_FLAG || flag == DATA_ACK_FLAG) {
			if(flag == DATA_FLAG) {
				mySocket = getSocket(destPort, srcPort);
				myTCPPack = (tcp_pack*)(myNewMsg.payload);
				
				if(myMsg->payload[0] != mySocket.rcvdBuff[seq]) {
					dbg(TRANSPORT_CHANNEL, "RECEIVED DATA %u\n", myMsg->payload[0]);
				}
				//Last Frame Received < Sequence Number <= Largest Acceptable Frame
				if(seq <= mySocket.largestAcceptable && seq > mySocket.lastRcvd) {
					//Buffer receive data
					mySocket.rcvdBuff[seq] = myMsg->payload[0];

					if(seq == mySocket.nextExpected) {
						SeqNumToAck = seq;
						mySocket.nextExpected++;
						//Largest Acceptable Frame - Last Frame Received <= Receive Window Size
						while((mySocket.largestAcceptable - seq) <= mySocket.effectiveWindow) {
							mySocket.largestAcceptable++;
						}
						
						dbg(TRANSPORT_CHANNEL, "READING DATA...\n");
						//Initializing ACK packet
						myTCPPack->ACK = SeqNumToAck;
						myTCPPack->seq = SeqNumToAck;
						myTCPPack->window = mySocket.largestAcceptable;
						myTCPPack->flag = DATA_ACK_FLAG;	
						myTCPPack->srcPort = mySocket.src.port;
						myTCPPack->destPort = mySocket.dest.port;
						call Transport.makePack(&myNewMsg, TOS_NODE_ID, mySocket.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
		
						call Sender.send(myNewMsg, (call RoutingTable.get(mySocket.dest.addr)));
					}
				}	
				activeSockets[mySocket.fd] = mySocket;		
			}else if(flag == DATA_ACK_FLAG) {
				//dbg(TRANSPORT_CHANNEL, "RECEIVED DATA ACK %hhu %hhu\n", lastAck, seq);
				mySocket = getSocket(destPort, srcPort);
				myTCPPack = (tcp_pack*)(myNewMsg.payload);
				call TimeoutTimer.stop();
				//SequenceNumToACK
				if(seq == lastAck) {
					dbg(TRANSPORT_CHANNEL, "ACK: %d\n", lastAck);
					mySocket.lastAck = lastAck;
					
					activeSockets[mySocket.fd] = mySocket;
					
					call SendPacketQueue.popfront();
					
				
				}
				
				//If all data has been read close socket
				if(lastAck == mySocket.transfer) {
					
					call SendPacketQueue.popfront();
					call TimeoutTimer.stop();
					dbg(TRANSPORT_CHANNEL, "CLIENT INITIATING TEARDOWN... \n");
					call Transport.close(mySocket);
				}
			}
		}else if(flag == FIN_FLAG || flag == FIN_ACK_FLAG){
			//Closing 
			if(flag == FIN_FLAG) {
				call SendPacketQueue.popfront();
				call TimeoutTimer.stop();
				mySocket = getSocket(srcPort, destPort);
				myTCPPack = (tcp_pack*)(myNewMsg.payload);
				dbg(TRANSPORT_CHANNEL, "RECEIVED FIN\n");
				call Transport.close(mySocket);
			}else if(flag == FIN_ACK_FLAG) {
				call SendPacketQueue.popfront();
				call TimeoutTimer.stop();
				mySocket = getSocket(srcPort, destPort);
				dbg(TRANSPORT_CHANNEL, "RECEIVED FIN ACK\n");
				call Transport.close(mySocket);
			}
		}
	}
     
	
  
  
   command error_t Transport.release(socket_fd_t fd){
   
   }
   command error_t Transport.listen(socket_fd_t fd){
   
   }

   command void Transport.setTestServer(){
	socket_t mySocket;
	socket_addr_t myAddr;
	uint8_t i = 0;

	socket_fd_t fd = call Transport.socket();

	myAddr.addr = TOS_NODE_ID;
	myAddr.port = 3;
	
	
	mySocket.fd = fd;
	
	mySocket.src = myAddr;
	mySocket.state = LISTEN;
	
	mySocket.transfer = 10;
	mySocket.effectiveWindow = 5;
	mySocket.largestAcceptable = mySocket.effectiveWindow;
	mySocket.nextExpected = 1;
	mySocket.lastRcvd = 0;	
	mySocket.lastRead = 0;

	for(i = 0; i < SOCKET_BUFFER_SIZE; i++) {
		mySocket.rcvdBuff[i] = 0;
	}
	activeSockets[fd] = mySocket;
	call Transport.accept(mySocket);
	
	dbg(ROUTING_CHANNEL, "CREATING SERVER AT %d ON PORT %d\n", mySocket.src.addr, mySocket.src.port);
}

   command void Transport.setTestClient() {
	socket_t mySocket;
	socket_fd_t fd;
	socket_addr_t myAddr;
	socket_addr_t serverAddr;
	uint8_t i = 0;

	fd = call Transport.socket();

	myAddr.addr = TOS_NODE_ID;
	myAddr.port = 2;

	//bind
	mySocket.fd = fd;

	serverAddr.addr = 2;
	serverAddr.port = 3;

	mySocket.src = myAddr;
	mySocket.dest = serverAddr;

	mySocket.lastSent = 0;
	mySocket.effectiveWindow = 5;
	mySocket.transfer = 10;
	mySocket.lastWritten = 0;
	mySocket.lastAck = 0;
	for(i = 0; i < SOCKET_BUFFER_SIZE; i++) {
		mySocket.sendBuff[i] = 0;
	}

	activeSockets[fd] = mySocket;
	dbg(ROUTING_CHANNEL, "CREATING CLIENT AT %d ON PORT %d\n", mySocket.src.addr, mySocket.src.port);

	call Transport.connect(mySocket);
	
}
	
   event void ClientWriteTimer.fired(){
	uint8_t i = 0;
	socket_t mySocket;
	
	//Search for correct socket
	for(i = 0; i < maxSockets; i++) {
		mySocket = activeSockets[i];
		if(mySocket.src.addr == TOS_NODE_ID) {
			break;
		}else if(i == maxSockets - 1) {
			//dbg(TRANSPORT_CHANNEL, "COULD NOT LOCATE SOCKET\n");
			return;
		}
	}
	//if sendBuff empty, fill with data
	if(mySocket.sendBuff[mySocket.transfer] == 0){
		for(i = 0; i < mySocket.transfer && i < SOCKET_BUFFER_SIZE; i++) {
			mySocket.sendBuff[i] = i + 1;
		}
	}
	//call clientWrite and update transfer index
	
	mySocket.lastWritten =  mySocket.lastWritten + (call Transport.write(mySocket, mySocket.sendBuff, SOCKET_BUFFER_SIZE));
	mySocket.transfer = mySocket.transfer - mySocket.lastWritten;

	if(mySocket.transfer == 0) {
		call ClientWriteTimer.stop();
	} 
}

   event void SendTimer.fired() {
	uint8_t i = 0;
	pack myMsg;
	tcp_pack* myTCPPack;
	socket_t mySocket;

	myMsg = call SendPacketQueue.front();
	myTCPPack = (tcp_pack *)(myMsg.payload);
	mySocket = getSocket(myTCPPack->srcPort, myTCPPack->destPort);
	i = myTCPPack->window;
	
	if(myTCPPack->flag == DATA_FLAG){
		while(i <= mySocket.effectiveWindow) {
			if(!(call SendPacketQueue.isEmpty())) {
				myMsg = call SendPacketQueue.get(i);
				call Sender.send(myMsg, (call RoutingTable.get(myMsg.dest)));
				i++;
				activeSockets[mySocket.fd] = mySocket;
			}
		}
	}else{
		if(!(call SendPacketQueue.isEmpty())) {
			myMsg = call SendPacketQueue.front();
			call Sender.send(myMsg, (call RoutingTable.get(myMsg.dest)));
			call TimeoutTimer.startOneShot(300000);
		}
	}
	dbg(TRANSPORT_CHANNEL, "\n");
}
   event void ServerReadTimer.fired(){
	pack myMsg;
	tcp_pack* myTCPPack;
	socket_t mySocket;
	
	if(!(call RcvdPacketQueue.isEmpty())) {
		
		myMsg = (call RcvdPacketQueue.popfront());
		myTCPPack = (tcp_pack *)(myMsg.payload);

		mySocket = getSocket(myTCPPack->destPort, myTCPPack->srcPort);
	
		mySocket.lastRead = mySocket.lastRead + (call Transport.read(mySocket, mySocket.rcvdBuff, SOCKET_BUFFER_SIZE));
		activeSockets[mySocket.fd] = mySocket;
	}

	if(mySocket.lastRead == mySocket.transfer) {
		myTCPPack->flag = FIN_FLAG;
		dbg(TRANSPORT_CHANNEL, "CLIENT INITIATING TEARDOWN...\n");
		call Transport.makePack(&myMsg, TOS_NODE_ID, mySocket.dest.addr, 20, PROTOCOL_TCP, 0, myTCPPack, PACKET_MAX_PAYLOAD_SIZE);
		call Sender.send(myMsg, mySocket.dest.addr);	
	}		
}

   event void TimeWait.fired() {
	socket_t mySocket;
	uint8_t i = 0;
	
	for(i = 0; i < maxSockets; i++){
		mySocket = activeSockets[i];
		if(mySocket.state == TIME_WAIT) {
			call Transport.close(mySocket);
		}
	}
}

   event void TimeoutTimer.fired() {
	pack myMsg;
	socket_t mySocket;
	tcp_pack* myTCPPack;
	call SendTimer.stop();
	dbg(TRANSPORT_CHANNEL, "TIMEOUT\n");
	if(!(call SendPacketQueue.isEmpty())) {
		
		if(myTCPPack->flag == SYN_FLAG || myTCPPack->flag == SYN_ACK_FLAG || myTCPPack->flag == ACK_FLAG) {
			dbg(TRANSPORT_CHANNEL, "RETRANSMITTING \n");
			call SendTimer.startOneShot(10);	
		}else {
			myMsg = call SendPacketQueue.front();
			myTCPPack = (tcp_pack *)(myMsg.payload);
			mySocket = getSocket(myTCPPack->srcPort, myTCPPack->destPort);
			dbg(TRANSPORT_CHANNEL, "RETRANSMITTING %d\n", myTCPPack->seq);
			
			call SendTimer.startOneShot(10);
		}
	}
}
   command void Transport.makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }
   
}
