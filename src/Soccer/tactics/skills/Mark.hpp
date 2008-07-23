#ifndef _TACTICS__MARK_H_
#define _TACTICS__MARK_H_

#include "../Tactics.hpp"
#include "../parameters/Robot_Parameter.hpp"
#include "../parameters/Float_Parameter.hpp"

namespace Tactics
{
    class Mark: public Base
    {
    public:
        Mark(Role *role);
        
        virtual void run();
        
    protected:
        Robot_Parameter opp_param;
        Float_Parameter distance_param;
    };
}

#endif // _TACTICS__MARK_H_
