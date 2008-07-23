#ifndef _TACTICS__HANDLER_H_
#define _TACTICS__HANDLER_H_

#include "Tactics.hpp"

namespace Tactics
{
    class Handler: public Base
    {
		public:
			Handler(Role *role);
			
			virtual void run();
			
			virtual float score(Robot* r);
			
			virtual void start() { _state = GetBall; }
			
			//handler is done when he has passed the ball
			virtual bool done() { return _state == Done; }
			
		private:
			typedef enum
			{
				GetBall,
				Pass,
				Done
			} State;
			
			State _state;
    };
}

#endif // _TACTICS__HANDLER_H_
