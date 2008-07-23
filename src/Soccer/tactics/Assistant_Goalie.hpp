#ifndef _TACTICS__ASSISTANT_GOALIE_H_
#define _TACTICS__ASSISTANT_GOALIE_H_

#include "Tactics.hpp"

namespace Tactics
{
    class Assistant_Goalie: public Base
    {
		public:
			Assistant_Goalie(Role *role);
			
			virtual float score(Robot* r);
			
			virtual void run();
			
		protected:
    };
}

#endif // _TACTICS__ASSISTANT_GOALIE_H_
