#ifndef KICK_HPP_
#define KICK_HPP_

#include "../Skill.hpp"

class Kick : public Skill
{
	public:
		Kick();
		
		virtual Packet::MotionCmd::Robot run();
};

#endif /* KICK_HPP_ */
