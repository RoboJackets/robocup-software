#ifndef _RECEIVER_HPP_
#define _RECEIVER_HPP_

#include <vector>

#include <Serialization.hpp>

namespace Serialization
{
    class Receiver
    {
    public:
        Receiver(uint16_t port, unsigned int size = 65536)
        {
            setup(0, port, size);
        }
        
        Receiver(const char *addr, uint16_t port, unsigned int size = 65536)
        {
            setup(addr, port, size);
        }
        
        ~Receiver();
    
        template<typename T>
        void receive(T &packet)
        {
            _buf.data.resize(_size);
            receive(_buf.data);
            
            ReadBuffer &reader = _buf;
            reader & (T &)packet;
        }
        
        void receive(std::vector<uint8_t> &data);
    
    protected:
        void setup(const char *addr, uint16_t port, unsigned int size);
        
        int _socket;
        
        // Maximum packet size
        unsigned int _size;
        
        MemoryBuffer _buf;
    };
}

#endif // _RECEIVER_HPP_
