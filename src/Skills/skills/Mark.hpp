#ifndef MARK_HPP_
#define MARK_HPP_

#include "../Skill.hpp"

class Mark : public Skill
{
	public:
		Mark();
		
		virtual Packet::MotionCmd::Robot run();
		
	private:
		
};

#endif /* MARK_HPP_ */
