#ifndef ORIENT_HPP_
#define ORIENT_HPP_

#include "../Skill.hpp"

#include <Geometry/Point2d.hpp>

class Orient : public Skill
{
	public:
		Orient();
		
		virtual Packet::MotionCmd::Robot run();
		
		virtual void start();
		
	private:
		/** recorded position of the ball */
		Geometry::Point2d _ball;
		
};

#endif /* ORIENT_HPP_ */
