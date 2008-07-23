#ifndef _TACTICS__MIDFIELDER_H_
#define _TACTICS__MIDFIELDER_H_

#include "Tactics.hpp"
#include "parameters/Robot_Parameter.hpp"
#include <Geometry/Point2d.hpp>

namespace Tactics
{
    class Midfielder: public Base
    {
        public:
	    Midfielder(Role *role);

	    virtual void run();

        protected:
            Robot_Parameter opp_param;


    };
}
#endif
