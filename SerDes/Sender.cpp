#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <stdexcept>

#include "Sender.hpp"

using namespace std;

Sender::Sender(const char *addr, uint16_t port)
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
    
    sin.sin_port = htons(port);
    memcpy(&sin.sin_addr.s_addr, ent->h_addr_list[0], 4);
    if (connect(_socket, (struct sockaddr *)&sin, sizeof(sin)))
    {
        throw runtime_error("Can't connect");
    }
}

Sender::~Sender()
{
    if (_socket)
    {
        close(_socket);
    }
}

void Sender::send(const uint8_t *data, unsigned int len)
{
    if (::send(_socket, data, len, 0) != (int)len)
    {
        throw runtime_error("Send failed");
    }
}
