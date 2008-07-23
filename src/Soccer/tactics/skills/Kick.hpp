#ifndef _TACTICS__KICKER_H_
#define _TACTICS__KICKER_H_

#include "../Tactics.hpp"

#include "../parameters/Point_Parameter.hpp"
#include "../parameters/Float_Parameter.hpp"

namespace Tactics
{
    class Kick: public Base
    {
		public:
			Kick(Role *role);
			
            virtual float score(Robot *robot);
			virtual void run();
			
			virtual void start() { _state = InitPos; }
			virtual bool done() { return _state == Done; }
			
		protected:		
			Point_Parameter _pos_param;
			Float_Parameter _strength;
			
		private:
			typedef enum
			{
				InitPos,
				KickBall,
				Done
			} State;
			
			State _state;
			
			/** maximum velocity of the ball */
			float _maxVel;
    };
}

#endif // _TACTICS__KICKER_H_
