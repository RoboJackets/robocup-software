#ifndef _TACTICS__PASS_H_
#define _TACTICS__PASS_H_

#include "../Tactics.hpp"
#include "../parameters/Point_Parameter.hpp"
#include "../parameters/Robot_Parameter.hpp"

#include <Geometry/Line2d.hpp>
#include <QTime>

namespace Tactics
{
    class Pass: public Base
    {
    public:
        Pass(Role *role);
        
        virtual float score(Robot *robot);
        virtual void start();
        virtual void run();
        virtual bool done();
        
    protected:
        enum
        {
            State_Get,          // Get the ball
            State_Orient,       // Face receiver
            State_Kick,         // Kick the ball towards receiver
            State_Moving,       // Ball is moving
            State_Done          // Ball has crossed threshold
        } state;
        
        Robot_Parameter _receiver_param;
        Geometry::Line2d _threshold;
        QTime _start_time;
    };
}

#endif // _TACTICS__PASS_H_
