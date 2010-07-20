#include "multicast.hpp"

#include <netdb.h>
#include <arpa/inet.h>

bool multicast_add(QAbstractSocket& socket, const char* addr)
{
	struct ip_mreqn mreq;
	memset(&mreq, 0, sizeof(mreq));
	mreq.imr_multiaddr.s_addr = inet_addr(addr);
	return setsockopt(socket.socketDescriptor(), IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == 0;
}
