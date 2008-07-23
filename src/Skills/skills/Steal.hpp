#ifndef STEAL_HPP_
#define STEAL_HPP_

#include "../Skill.hpp"
#include "GotoBall.hpp"

class Steal : public Skill
{
	public:
		Steal();
		
		virtual Packet::MotionCmd::Robot run();
		
		virtual void start();
		
	private:
		typedef enum
		{
			InitPos,
			Spin,
			Done
		} State;
		
		State _state;
		
		unsigned int _spinCount;
		
		GotoBall _gotoBall;
};

#endif /* STEAL_HPP_ */
