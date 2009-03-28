#ifndef WIN32
# include <errno.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <arpa/inet.h>
#else
# include <windows.h>
# include <winsock.h>
# pragma comment(lib, "wsock32.lib")
#endif
#include <cstdio>
#include <cstring>
#include "udp_broadcast.h"

UDP_Broadcast::UDP_Broadcast() throw (UDP_Broadcast::IOError)
{
    // create socket
#ifndef WIN32
    sock = socket( AF_INET, SOCK_DGRAM, 0 );
#else
    WORD sockVersion;
    WSADATA wsaData;
    sockVersion = MAKEWORD(1,1);
    WSAStartup(sockVersion, &wsaData);
    sock = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
#endif    
    if ( sock == -1 )
    {
        perror("could not create socket\n");
        throw UDP_Broadcast::IOError("Could not create socket", errno);
    }

}

UDP_Broadcast::~UDP_Broadcast()
{
#ifndef WIN32
    close(sock);
#endif
}


void UDP_Broadcast::set_destination(const std::string& host,
                                    const uint16_t port) throw (UDP_Broadcast::IOError)
{
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
#ifndef WIN32
    if (! inet_aton(host.c_str(), &addr.sin_addr))
    {
        throw IOError("inet_aton failed. Invalid address: " + host);
    }
#else
    addr.sin_addr.s_addr = inet_addr(host.c_str());
    if (addr.sin_addr.s_addr == 0)
    {
        throw IOError("Invalid address: " + host);
    } 
#endif
    int yes = 1;
//     if (-1 == setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)))
//     {
//         throw IOError("could not set broadcast mode", errno);
//     }
    // allow packets to be received on this host        
#ifndef WIN32
    if (-1 == setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, 
                         (const char*)&yes, sizeof(yes)))
    {
        throw IOError("could not set multicast loop", errno);
    }
#endif

    if (-1 == connect(sock, (const struct sockaddr*) &addr, sizeof(addr)))
    {
        throw IOError("connect failed (" + host + ")", errno);
    }
    
    
}

void UDP_Broadcast::send(const void* buffer, const size_t buflen) throw (IOError)
{
    ssize_t result = ::send(sock, (const char*)buffer, buflen, 0);
    if (result == -1)
    {
        throw UDP_Broadcast::IOError(
            std::string("send() failed: ") + strerror(errno) );
    }   
}

void UDP_Broadcast::send(const std::string& buffer) throw (IOError)
{
    send(buffer.c_str(), buffer.size());
}

