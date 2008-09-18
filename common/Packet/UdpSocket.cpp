#include "UdpSocket.hpp"

#include <netinet/in.h>

using namespace Packet;

UdpSocket::UdpSocket()
	: QUdpSocket()
{
	
}

bool UdpSocket::join(QHostAddress addr)
{
	struct ip_mreq imreq;
	
	imreq.imr_multiaddr.s_addr = htonl(addr.toIPv4Address());
	imreq.imr_interface.s_addr = INADDR_ANY;
    
	return (setsockopt(socketDescriptor(), IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(imreq)) == 0);
}
