#include <netdb.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>
#include <stdexcept>

#include "Sender.hpp"

using namespace std;

Network::Sender::Sender(const char *addr, uint16_t port)
{
    _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socket < 0)
    {
        throw runtime_error("Can't create socket");
    }

    struct sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = 0;
    sin.sin_addr.s_addr = 0;
    if (bind(_socket, (struct sockaddr *)&sin, sizeof(sin)))
    {
        throw runtime_error("Can't bind");
    }

    struct hostent *ent = gethostbyname(addr);
    if (!ent)
    {
        throw runtime_error("Can't lookup destination address");
    }

    // Can't use connect() because it doesn't work if there are no network
    // interfaces except lo, even though sendto would not fail.
    _dest.sin_family = AF_INET;
    _dest.sin_port = htons(port);
    memcpy(&_dest.sin_addr.s_addr, ent->h_addr_list[0], 4);
}

Network::Sender::~Sender()
{
    if (_socket)
    {
        close(_socket);
    }
}

void Network::Sender::send(const uint8_t *data, unsigned int len)
{
    if (sendto(_socket, data, len, 0, (struct sockaddr *)&_dest, sizeof(_dest)) != (int)len)
    {
        throw runtime_error(strerror(errno));
    }
}
