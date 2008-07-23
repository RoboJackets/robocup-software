#ifndef _TACTICS__DRIBBLER_H_
#define _TACTICS__DRIBBLER_H_

#include "../Tactics.hpp"
#include "../parameters/Point_Parameter.hpp"

namespace Tactics
{
    class Dribble: public Base
    {
		public:
			Dribble(Role *role);
			
			virtual void run();
			
			/** dribbler is never done */
			virtual bool done() { return false; }
			
		protected:		
			Point_Parameter _pos1, _pos2;
			
		private:
			typedef enum
			{
				NextLoc,
				AcquireBall,
				Reorient,
				Travel
			} State;
			
			State _state;
			
			/** next location to take the ball to */
			Geometry::Point2d _nextLoc;
    };
}

#endif // _TACTICS__DRIBBLER_H_
