#include "robocup-py.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <string>
#include <sstream>

using namespace boost::python;

#include <Geometry2d/Point.hpp>
#include <Robot.hpp>
#include <SystemState.hpp>
#include <protobuf/LogFrame.pb.h>


//	this is here so boost can work with std::shared_ptr
template<class T> T * get_pointer( std::shared_ptr<T> const& p) {
	return p.get();
}

std::string Point_repr(Geometry2d::Point *thiss) {
	std::ostringstream ss;
	ss << "Point(";
	ss << thiss->x;
	ss << ", ";
	ss << thiss->y;
	ss << ")";
	
	std::string repr(ss.str());
	return repr;
}

std::string Robot_repr(Robot *thiss) {
	std::ostringstream ss;
	ss << (thiss->self() ? "us[" : "them[");
	ss << thiss->shell();
	ss << "], pos=";
	ss << Point_repr(&(thiss->pos));

	std::string repr(ss.str());
	return repr;
}


void OurRobot_move_to(OurRobot *thiss, Geometry2d::Point *to) {
	thiss->move(*to);
}


/**
 * The code in this block wraps up c++ classes and makes them
 * accessible to python in the 'robocup' module.
 */
BOOST_PYTHON_MODULE(robocup)
{
	class_<Geometry2d::Point>("Point", init<float, float>())
		.def_readwrite("x", &Geometry2d::Point::x)
		.def_readwrite("y", &Geometry2d::Point::y)
		.def(self - self)
		.def(self + self)
		.def("mag", &Geometry2d::Point::mag)
		.def("__repr__", &Point_repr)
	;

	class_<Geometry2d::Segment>("Segment", init<Geometry2d::Point, Geometry2d::Point>())
		.def("center", &Geometry2d::Segment::center)
	;

	//		I'm holding off for now because GameState needs some attention on the C++
	//		side of things before we spread its shortcomings into the python world too...
	class_<GameState>("GameState")
		.def_readonly("our_score", &GameState::ourScore)
		.def_readonly("their_score", &GameState::theirScore)
		.def("is_halted", &GameState::halt)
		.def("is_stopped", &GameState::stopped)
		.def("is_playing", &GameState::playing)
		.def("is_kickoff", &GameState::kickoff)
		.def("is_penalty", &GameState::penalty)
		.def("is_direct", &GameState::direct)
		.def("is_indirect", &GameState::indirect)
		.def("is_our_kickoff", &GameState::ourKickoff)
		.def("is_our_penalty", &GameState::ourPenalty)
		.def("is_our_direct", &GameState::ourDirect)
		.def("is_our_indirect", &GameState::ourIndirect)
		.def("is_our_free_kick", &GameState::ourFreeKick)
		.def("is_their_kickoff", &GameState::theirKickoff)
		.def("is_their_penalty", &GameState::theirPenalty)
		.def("is_their_direct", &GameState::theirDirect)
		.def("is_their_indirect", &GameState::theirIndirect)
		.def("is_their_free_kick", &GameState::theirFreeKick)
		.def("is_setup_restart", &GameState::setupRestart)
		.def("can_kick", &GameState::canKick)
		.def("stay_away_from_ball", &GameState::stayAwayFromBall)
		.def("stay_on_side", &GameState::stayOnSide)
		.def("stay_behind_penalty_line", &GameState::stayBehindPenaltyLine)
	;

	class_<Robot>("Robot", init<int, bool>())
		.def("shell_id", &Robot::shell)
		.def("is_ours", &Robot::self)
		.def_readwrite("pos", &Robot::pos)
		.def_readwrite("vel", &Robot::vel)
		.def_readwrite("angle", &Robot::angle)
		.def_readwrite("angle_vel", &Robot::angleVel)
		.def("__repr__", &Robot_repr);
	;

	class_<OurRobot, OurRobot *, std::shared_ptr<OurRobot>, bases<Robot> >("OurRobot", init<int, SystemState*>())
		.def("move_to", &OurRobot_move_to)
	;

	class_<OpponentRobot, std::shared_ptr<OpponentRobot>, bases<Robot> >("OpponentRobot", init<int>());

	class_<Ball, std::shared_ptr<Ball> >("Ball", init<>())
		.def_readonly("pos", &Ball::pos)
		.def_readonly("vel", &Ball::vel)
		.def_readonly("valid", &Ball::valid)
	;

	class_<std::vector<OurRobot *> >("vector_OurRobot")
		.def(vector_indexing_suite<std::vector<OurRobot *> >())
	;

	class_<std::vector<OpponentRobot *> >("vector_OpponentRobot")
		.def(vector_indexing_suite<std::vector<OpponentRobot *> >())
	;

	class_<SystemState>("SystemState")
		.def_readonly("our_robots", &SystemState::self)
		.def_readonly("their_robots", &SystemState::opp)
		.def_readonly("ball", &SystemState::ball)
		.def_readonly("game_state", &SystemState::gameState)
		.def_readonly("timestamp", &SystemState::timestamp)

		//	debug drawing methods
		.def("draw_circle", &SystemState::drawCircle)
		.def("draw_path", &SystemState::drawPath)
		.def("draw_text", &SystemState::drawText)
		.def("draw_obstacle", &SystemState::drawObstacle)
		.def("draw_obstacles", &SystemState::drawObstacles)
	;
}
