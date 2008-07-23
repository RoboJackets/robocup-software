#ifndef RECEIVE_HPP_
#define RECEIVE_HPP_

#include "../Skill.hpp"
#include "GotoBall.hpp"

class Receive : public Skill
{
	public:
		Receive();
		
		virtual Packet::MotionCmd::Robot run();
		
		virtual void start();
		
	private:
       GotoBall _gotoBall;
};

#endif /* RECEIVE_HPP_ */
