#pragma once

#include "Entity.hpp"

class Ball: public Entity
{
	public:
		Ball(Env* env);
		virtual ~Ball();

		virtual void position(float x, float y);
        void velocity(float x, float y);
};
