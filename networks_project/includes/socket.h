#ifndef __SOCKET_H__
#define __SOCKET_H__

#define CLOSED 0
#define FIN_WAIT 1
#define TIME_WAIT 2
#define LISTEN 3
#define ESTABLISHED 4
#define SYN_SENT 5
#define SYN_RCVD 6

enum{
    MAX_NUM_OF_SOCKETS = 10,
    ROOT_SOCKET_ADDR = 255,
    ROOT_SOCKET_PORT = 255,
    SOCKET_BUFFER_SIZE = 128,
};

/*enum socket_state{
    CLOSED,
    FIN_WAIT,
    TIME_WAIT,
    LISTEN,
    ESTABLISHED,
    SYN_SENT,
    SYN_RCVD,
};*/


typedef nx_uint8_t nx_socket_port_t;
typedef uint8_t socket_port_t;

// socket_addr_t is a simplified version of an IP connection.
typedef nx_struct socket_addr_t{
    nx_socket_port_t port;
    nx_uint16_t addr;
}socket_addr_t;


// File descripter id. Each id is associated with a socket_store_t
typedef uint8_t socket_fd_t;

// State of a socket. 
typedef struct socket_t{
    uint8_t fd;
    uint8_t flag;
    //enum socket_state state;
    uint8_t state;
    socket_addr_t src;
    socket_addr_t dest;

    // This is the sender portion.
    uint8_t sendBuff[SOCKET_BUFFER_SIZE];
    uint8_t lastWritten;
    uint8_t lastAck;
    uint8_t lastSent;

    // This is the receiver portion
    uint8_t rcvdBuff[SOCKET_BUFFER_SIZE];
    uint8_t lastRead;
    uint8_t lastRcvd;
    uint8_t largestAcceptable;
    uint8_t nextExpected;

    uint16_t RTT;
    uint8_t effectiveWindow;
    uint8_t transfer;
}socket_t;

#endif
