#ifndef _BALL_HPP
#define _BALL_HPP

#include "Entity.hpp"

class Ball: public Entity
{
	public:
		Ball(NxScene& scene);
		~Ball();

		virtual void position(float x, float y);
		
	private:
};

#endif /* _BALL_HPP */
