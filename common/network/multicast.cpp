#include <arpa/inet.h>
#include <network/multicast.h>

#include <cstring>

// bool multicast_add(QAbstractSocket* socket, const char* addr) {
//    return multicast_add_native(socket->socketDescriptor(), addr);
//}

bool multicast_add_native(int socket, const char* addr) {
    struct ip_mreqn mreq;
    std::memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(addr);
    return setsockopt(socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                      sizeof(mreq)) == 0;
}
