#ifndef _TACTICS__ORIENT_H_
#define _TACTICS__ORIENT_H_

#include "../Tactics.hpp"
#include "../parameters/Point_Parameter.hpp"

#include <Geometry/Point2d.hpp>

namespace Tactics
{
    class Orient: public Base
    {
		public:
			Orient(Role *role);
			
            virtual float score(Robot *robot);
			virtual void run();
			virtual bool done();
			
		private:
            Point_Parameter _pos_param;
    };
}

#endif // _TACTICS__KICKER_H_
