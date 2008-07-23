#ifndef BALL_HPP_
#define BALL_HPP_

#include "Entity.hpp"

namespace Physics
{
	class Ball : public Entity
	{
		public:
			Ball(dWorldID world, dSpaceID space);
			~Ball();
			
			void paint();
			
			virtual void internal();
			
			float radius() const { return _radius; }
			
		private:
            GLUquadric *_quadric;
			static const float _radius;
	};
}

#endif /*BALL_HPP_*/
