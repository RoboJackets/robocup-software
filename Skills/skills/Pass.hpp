#ifndef PASS_HPP_
#define PASS_HPP_

#include "../Skill.hpp"

class Pass : public Skill
{
	public:
		Pass();
		
		virtual Packet::MotionCmd::Robot run();
		
	private:
};

#endif /* PASS_HPP_ */
