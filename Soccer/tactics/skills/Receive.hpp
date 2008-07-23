#ifndef _TACTICS__RECEIVE_H_
#define _TACTICS__RECEIVE_H_

#include "../Tactics.hpp"
#include "../parameters/Point_Parameter.hpp"
#include "../parameters/Robot_Parameter.hpp"

namespace Tactics
{
    class Receive: public Base
    {
    public:
        Receive(Role *role);
        
        virtual void start();
        virtual void run();
        virtual bool done();
    };
}

#endif // _TACTICS__RECEIVE_H_
