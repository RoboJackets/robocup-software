#include "Receive_Thread.hpp"

#include <QMutexLocker>

#include <Packet/PacketReceiver.hpp>

using namespace Packet;

void Receive_Thread::grab()
{
    QMutexLocker ml(&_mutex);
    
    Record::Entry entry;
    for (int i = 0; i < 5; ++i)
    {
        record.valid[i] |= _state[i].valid;
        entry.pos[i] = _state[i].pos;
    }
    record.records.push_back(entry);
}

void Receive_Thread::vision_receiver(const VisionData &vision)
{
    QMutexLocker ml(&_mutex);
    for (int i = 0; i < 5; ++i)
    {
        _state[i].valid = vision.self[i].valid;
        if (vision.self[i].valid)
        {
            _state[i].pos = vision.self[i].pos;
        }
    }
}

void Receive_Thread::run()
{
    PacketReceiver receiver;
    receiver.addType(team, this, &Receive_Thread::vision_receiver);
    
    _running = true;
    while (_running)
    {
        receiver.receive();
    }
}

void Receive_Thread::stop()
{
    _running = false;
    wait();
}
