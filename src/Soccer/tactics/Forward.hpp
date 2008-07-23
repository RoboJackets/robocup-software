#ifndef _TACTICS__FORWARD_H_
#define _TACTICS__FORWARD_H_

#include "Tactics.hpp"
#include "parameters/Robot_Parameter.hpp"

namespace Tactics
{
    class Forward: public Base
    {
		public:
			Forward(Role *role);

			virtual void run();

		protected:
			Robot_Parameter _opp;
    };
}

#endif // _TACTICS__FORWARD_H_
