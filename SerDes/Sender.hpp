#ifndef _SENDER_HPP_
#define _SENDER_HPP_

#include <vector>

#include "Serialization.hpp"

class Sender
{
public:
    Sender(const char *addr, uint16_t port);
    ~Sender();

    Serialization::MemoryBuffer buf;
    
    template<typename T>
    void send(const T &packet)
    {
        // Erase data but keep capacity to avoid reallocation.
        buf.data.resize(0);
        Serialization::WriteBuffer &writer = buf;
        writer & (T &)packet;
        printf("send %d\n", buf.data.size());
        //send(&buf.data[0], buf.data.size());
    }
    
    void send(const uint8_t *data, unsigned int len);

protected:
    int _socket;
};

#endif // _SENDER_HPP_
