#ifndef GOTOBALL_HPP_
#define GOTOBALL_HPP_

#include "../Skill.hpp"

class GotoBall : public Skill
{
	/// types ///
	private:
		typedef enum
		{
			InitPos,
			Approach
		} State;
	
	/// methods ///
	public:
		GotoBall();
		
		virtual Packet::MotionCmd::Robot run();
		
		virtual void start();
	
	/// members ///
	private:
		State _state;
		
		unsigned int _settleCount;
};

#endif /* GOTOBALL_HPP_ */
