#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include "UDP_Broadcast.hpp"

/// UDP_Broadcast Class ///

UDP_Broadcast::UDP_Broadcast() throw (UDP_Broadcast::IOError)
{
    /** create socket */
    sock = socket( AF_INET, SOCK_DGRAM, 0 );
  
    /** check if socket creation successful */
    if ( sock == SocketCreationError )
    {
        perror("could not create socket\n");
        throw UDP_Broadcast::IOError("Could not create socket", errno);
    }

}

UDP_Broadcast::~UDP_Broadcast()
{
    close(sock);
}

/** This function establishes a socket connection.
 *  AF_INET = 'Internet domain sockets'
 */
void UDP_Broadcast::setDestination(const std::string &host, const uint16_t port) throw (UDP_Broadcast::IOError)
{
    int yes = 1;
    
    struct sockaddr_in socketAddress;    
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_port = htons(port);

    /** convert Internet dot address to network address */
    if (! inet_aton(host.c_str(), &socketAddress.sin_addr))
    {
        throw IOError("inet_aton failed. Invalid address: " + host);
    }
    
    /** set the socket options to control socket behaviour */
    if (SetSocketOptionError == setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&yes, sizeof(yes)))
    {
        throw IOError("could not set multicast loop", errno);
    }
    
    /** request a connection to be made on a socket */
    if (ConnectionError == connect(sock, (const struct sockaddr*) &socketAddress, sizeof(socketAddress)))
    {
        throw IOError("connect failed (" + host + ")", errno);
    }
        
}

/** This function sends a packet */
void UDP_Broadcast::sendPacket(const void *buffer, const size_t bufferLength) throw (IOError)
{
    ssize_t result = ::send(sock, (const char*)buffer, bufferLength, 0);
    if (result == SendFailure)
    {
        throw UDP_Broadcast::IOError(std::string("send() failed: ") + strerror(errno) );
    }   
}

/** This function sends a packet */
void UDP_Broadcast::sendPacket(const std::string &buffer) throw (IOError)
{
    sendPacket(buffer.c_str(), buffer.size());
}

