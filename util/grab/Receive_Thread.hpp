#ifndef _RECEIVE_THREAD_HPP_
#define _RECEIVE_THREAD_HPP_

#include "Record.hpp"

#include <QThread>
#include <QMutex>

#include <Geometry/Point2d.hpp>
#include <Packet/VisionData.hpp>
#include <Team.h>

class Receive_Thread: public QThread
{
public:
    typedef struct
    {
        bool valid;
        Geometry::Point2d pos;
    } Robot_State;

    Team team;
    
    virtual void run();
    
    void stop();
    
    void grab();
    void write(FILE *fp);
    
    Record record;
    
protected:
    volatile bool _running;
    QMutex _mutex;
    Robot_State _state[5];
    
    void vision_receiver(const Packet::VisionData &vision);
};

#endif // _RECEIVE_THREAD_HPP_
