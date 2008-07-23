#ifndef _TACTICS__STEAL_H_
#define _TACTICS__STEAL_H_

#include "../Tactics.hpp"
#include "../parameters/Robot_Parameter.hpp"

namespace Tactics
{
    class Steal: public Base
    {
		public:
			Steal(Role *role);
			
			virtual void run();
			
			virtual bool done();
			
		private:
			Robot_Parameter _opp;
    };
}

#endif // _TACTICS__DRIBBLER_H_
