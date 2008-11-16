#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdexcept>
#include <string.h>

#include "Receiver.hpp"

#include <Serialization.hpp>

using namespace std;

void Network::Receiver::setup(const char *addr, uint16_t port, unsigned int size)
{
    _size = size;

    _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socket < 0)
    {
        throw runtime_error("Can't create socket");
    }

    int value = 1;
    if (setsockopt(_socket, SOL_SOCKET, SO_REUSEADDR, &value, sizeof(value)) < 0)
    {
        throw runtime_error("Failed to set SO_REUSEADDR");
    }

    struct sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(port);
    sin.sin_addr.s_addr = 0;
    if (bind(_socket, (struct sockaddr *)&sin, sizeof(sin)))
    {
        throw runtime_error("Can't bind");
    }

    if (addr)
    {
        struct ip_mreqn mreq;
        memset(&mreq, 0, sizeof(mreq));

        mreq.imr_multiaddr.s_addr = inet_addr(addr);
        setsockopt(_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    }
}

Network::Receiver::~Receiver()
{
    if (_socket)
    {
        close(_socket);
    }
}

void Network::Receiver::receive(vector<uint8_t> &data)
{
    int len = recv(_socket, &data[0], data.size(), 0);
    if (len < 0)
    {
        throw runtime_error("Receive failed");
    }
    data.resize(len);
}
