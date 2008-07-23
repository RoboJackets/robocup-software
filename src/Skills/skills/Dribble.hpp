#ifndef DRIBBLE_HPP_
#define DRIBBLE_HPP_

#include "../Skill.hpp"

class Dribble : public Skill
{
	public:
		Dribble();
		
		virtual Packet::MotionCmd::Robot run();
		
	private:
};

#endif /* DRIBBLE_HPP_ */
