#ifndef _RECEIVER_HPP_
#define _RECEIVER_HPP_

#include <vector>

#include "Serialization.hpp"

class Receiver
{
public:
    Receiver(uint16_t port, unsigned int size);
    ~Receiver();

    template<typename T>
    void receive(T &packet)
    {
        _buf.data.resize(_size);
        receive(_buf.data);
        
        Serialization::ReadBuffer &reader = _buf;
        reader & (T &)packet;
    }
    
    void receive(std::vector<uint8_t> &data);

protected:
    int _socket;
    
    // Maximum packet size
    unsigned int _size;
    
    Serialization::MemoryBuffer _buf;
};

#endif // _RECEIVER_HPP_
