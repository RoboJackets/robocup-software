#ifndef _STATUS_THREAD_HPP_
#define _STATUS_THREAD_HPP_

#include "Command_Thread.hpp"

#include <Team.h>
#include <Packet/RobotStatus.hpp>

#include <QThread>

class Radio;
class Command_Thread;

class Status_Thread: public QThread
{
public:
    Status_Thread(Command_Thread *ct, Team team);
    
    void stop();
    void send();
    
protected:
    Radio *_radio;
    Command_Thread *_command_thread;
    Team _team;
    bool _running;
    Packet::Sender<Packet::RobotStatus> *_sender;
    
    int _last_sequence[Command_Thread::Num_Robots];
    
    virtual void run();
    void handle_packet(const uint8_t *packet);
};

#endif // _STATUS_THREAD_HPP_
