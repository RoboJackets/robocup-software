#ifndef NONE_HPP_
#define NONE_HPP_

#include "../Skill.hpp"

class None : public Skill
{
	public:
		None();
		
		virtual Packet::MotionCmd::Robot run();
		
	private:
};

#endif /* NONE_HPP_ */
