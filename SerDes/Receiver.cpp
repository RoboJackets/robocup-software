#include <sys/socket.h>
#include <netinet/in.h>

#include <stdexcept>

#include "Receiver.hpp"

using namespace std;

Receiver::Receiver(uint16_t port)
{
    _socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_socket < 0)
    {
        throw runtime_error("Can't create socket");
    }
    
    struct sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port = htons(port);
    sin.sin_addr.s_addr = 0;
    if (bind(_socket, (struct sockaddr *)&sin, sizeof(sin)))
    {
        throw runtime_error("Can't bind");
    }
}

Receiver::~Receiver()
{
    if (_socket)
    {
        close(_socket);
    }
}

void Receiver::receive(vector<uint8_t> &data)
{
    int len = recv(_socket, &data[0], data.size(), 0);
    if (len < 0)
    {
        throw runtime_error("Receive failed");
    }
    data.resize(len);
}
