#ifndef _RECEIVER_HPP_
#define _RECEIVER_HPP_

#include <vector>

class Receiver
{
public:
    Receiver(uint16_t port);
    ~Receiver();

    void receive(std::vector<uint8_t> &data);

protected:
    int _socket;
};

#endif // _RECEIVER_HPP_
