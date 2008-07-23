#ifndef _TACTICS__MOVE_H_
#define _TACTICS__MOVE_H_

#include "../Tactics.hpp"
#include "../parameters/Point_Parameter.hpp"
#include "../parameters/Float_Parameter.hpp"

namespace Tactics
{
    class Move: public Base
    {
    public:
        Move(Role *role);
        
        virtual void run();
        virtual bool done();
        
        virtual float score(Robot* robot);
        
    protected:
        Point_Parameter pos_param;
        Point_Parameter face_param;
        Float_Parameter threshold_param;
    };
}

#endif // _TACTICS__MOVE_H_
