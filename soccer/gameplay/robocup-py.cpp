#include "robocup-py.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <string>
#include <sstream>
#include <iostream>

using namespace boost::python;

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Rect.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Arc.hpp>
#include <Robot.hpp>
#include <SystemState.hpp>
#include <protobuf/LogFrame.pb.h>
#include <Constants.hpp>
#include <WindowEvaluator.h>

#include <boost/python/exception_translator.hpp>
#include <boost/version.hpp>
#include <exception>

#include "motion/TrapezoidalMotion.hpp"

/**
 * These functions make sure errors on the c++
 * side get passed up through python.
 */

struct NullArgumentException : public std::exception {
	std::string argument_name;
	NullArgumentException() : argument_name("") {}
	NullArgumentException(std::string name) : argument_name(name) {}
	virtual const char* what() const throw(){ return ("'" + argument_name + "'' was 'None'.").c_str(); }
};

void translateException(NullArgumentException const& e)
{
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

/**
 * NOTES FOR WRAPPER FUNCTIONS/METHODS
 *
 * Keep in mind that pointer parameters will be be nullptr/NULL if the value
 * from python was None.  Check for this case so that we don't segfault.
 */


//	this is here so boost can work with std::shared_ptr
//	later versions of boost include this, so we only define it for older versions
#if BOOST_VERSION < 105300
template<class T> T * get_pointer( std::shared_ptr<T> const& p) {
	return p.get();
}
#endif

QColor Color_from_tuple(const boost::python::tuple &rgb) {
	float r = extract<float>(rgb[0]);
	float g = extract<float>(rgb[1]);
	float b = extract<float>(rgb[2]);

	if (len(rgb) == 4) {
		float a = extract<float>(rgb[3]);
		return QColor(r, g, b, a);
	} else {
		return QColor(r, g, b);
	}
}

std::string Point_repr(Geometry2d::Point *self) {
	return self->toString();
}

std::string Robot_repr(Robot *self) {
	return self->to_string();
}

Geometry2d::Point Robot_pos(Robot *self) {
	return self->pos;
}

Geometry2d::Point Robot_vel(Robot *self) {
	return self->vel;
}

float Robot_angle(Robot *self) {
	return self->angle;
}

float Robot_angle_vel(Robot *self) {
	return self->angleVel;
}

void OurRobot_move_to_end_vel(OurRobot *self, Geometry2d::Point *endPos, Geometry2d::Point *vf) {
	if(endPos == nullptr || vf == nullptr)
		throw NullArgumentException();
	self->move(*endPos, *vf);
}

void OurRobot_move_to(OurRobot *self, Geometry2d::Point *to) {
	if(to == nullptr)
		throw NullArgumentException("to");
	self->move(*to);
}

void OurRobot_move_to_direct(OurRobot *self, Geometry2d::Point *to) {
	if(to == nullptr)
		throw NullArgumentException("to");
	self->moveDirect(*to);
}

void OurRobot_add_local_obstacle(OurRobot *self, Geometry2d::Shape *obs) {
	if(obs == nullptr)
		throw NullArgumentException("obs");
	std::shared_ptr<Geometry2d::Shape> sharedObs(obs->clone());
	self->localObstacles(sharedObs);
}

void OurRobot_set_avoid_ball_radius(OurRobot *self, float radius) {
	self->avoidBallRadius(radius);
}

void OurRobot_set_avoid_teammate_radius(OurRobot *self, unsigned shellID, float radius) {
	self->avoidTeammateRadius(shellID, radius);
}

void OurRobot_set_max_angle_speed(OurRobot *self, float maxAngleSpeed) {
	self->motionConstraints().maxAngleSpeed = maxAngleSpeed;
}

void OurRobot_approach_opponent(OurRobot *self, unsigned shell_id, bool enable_approach) {
	self->approachOpponent(shell_id, enable_approach);
}

void OurRobot_add_text(OurRobot *self, const std::string &text, boost::python::tuple rgb, const std::string &layerPrefix) {
	self->addText(QString::fromStdString(text), Color_from_tuple(rgb), QString::fromStdString(layerPrefix));
}

void OurRobot_set_avoid_opponents(OurRobot *self, bool value) {
	self->avoidOpponents(value);
}

bool Rect_contains_rect(Geometry2d::Rect *self, Geometry2d::Rect *other) {
	if(other == nullptr)
		throw NullArgumentException("other");
	return self->contains(*other);
}

bool Rect_contains_point(Geometry2d::Rect *self, Geometry2d::Point *pt) {
	if(pt == nullptr)
		throw NullArgumentException("pt");
	return self->contains(*pt);
}

void Point_rotate(Geometry2d::Point *self, Geometry2d::Point *origin, float angle) {
	if(origin == nullptr)
		throw NullArgumentException("origin");
	self->rotate(*origin, angle);
}

void CompositeShape_add_shape(Geometry2d::CompositeShape *self, Geometry2d::Shape *shape) {
	if(shape == nullptr)
		throw NullArgumentException("shape");
	self->add(std::shared_ptr<Geometry2d::Shape>( shape->clone() ));
}

void Polygon_add_vertex(Geometry2d::Polygon *self, Geometry2d::Point *pt) {
	if(pt == nullptr)
		throw NullArgumentException("pt");
	self->addVertex(*pt);
}

Geometry2d::Point* Line_get_pt(Geometry2d::Line *self, int index) {
	return &(self->pt[index]);
}

Geometry2d::Point* Rect_get_pt(Geometry2d::Rect *self, int index) {
	return &(self->pt[index]);
}

boost::python::object Segment_segment_intersection(Geometry2d::Segment *self, Geometry2d::Segment *other) {
	if(other == nullptr)
		throw NullArgumentException("other");
	Geometry2d::Point pt;
	if (self->intersects(*other, &pt)) {
		boost::python::object obj(pt);
		return obj;
	} else {
		//	return None
		return boost::python::object();
	}
}

boost::python::object Segment_line_intersection(Geometry2d::Segment *self, Geometry2d::Line *line) {
	if(line == nullptr)
		throw NullArgumentException{"line"};
	Geometry2d::Point pt;
	if(self->intersects(*line, &pt)) {
		boost::python::object obj{pt};
		return obj;
	} else {
		return boost::python::object{};
	}
}

boost::python::object Segment_nearest_point_to_point(Geometry2d::Segment *self, Geometry2d::Point *point) {
	if(point == nullptr)
		throw NullArgumentException{"point"};
	return boost::python::object{self->nearestPoint(*point)};
}

boost::python::object Segment_nearest_point_to_line(Geometry2d::Segment *self, Geometry2d::Line *line) {
	if(line == nullptr)
		throw NullArgumentException{"line"};
	return boost::python::object{self->nearestPoint(*line)};
}

//	returns None or a Geometry2d::Point
boost::python::object Line_line_intersection(Geometry2d::Line *self, Geometry2d::Line *other) {
	if(other == nullptr)
		throw NullArgumentException("other");
	Geometry2d::Point pt;
	if (self->intersects(*other, &pt)) {
		boost::python::object obj(pt);
		return obj;
	} else {
		//	return None
		return boost::python::object();
	}
};

boost::python::tuple Line_intersects_circle(Geometry2d::Line *self, Geometry2d::Circle *circle) {
	if(circle == nullptr)
		throw NullArgumentException("circle");
	Geometry2d::Point a, b;
	boost::python::list lst;

	bool intersects = self->intersects(*circle, &a, &b);
	lst.append(intersects);
	lst.append(a);
	lst.append(b);

	return boost::python::tuple(lst);
}

void State_draw_circle(SystemState *self, const Geometry2d::Point *center, float radius, boost::python::tuple rgb, const std::string &layer) {
	if(center == nullptr)
		throw NullArgumentException("center");
	self->drawCircle(*center, radius, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_arc(SystemState *self, const Geometry2d::Arc *arc, boost::python::tuple rgb, const std::string &layer) {
	if(arc == nullptr)
		throw NullArgumentException{"arc"};
	self->drawArc(*arc, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_line(SystemState *self, const Geometry2d::Line *line, boost::python::tuple rgb, const std::string &layer) {
	if(line == nullptr)
		throw NullArgumentException("line");
	self->drawLine(*line, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_segment(SystemState *self, const Geometry2d::Point *p0, const Geometry2d::Point *p1, boost::python::tuple rgb, const std::string &layer) {
	if(p0 == nullptr)
		throw NullArgumentException{"p0"};
	if(p1 == nullptr)
		throw NullArgumentException{"p1"};
	self->drawLine(*p0, *p1, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_text(SystemState *self, const std::string &text, Geometry2d::Point *pos, boost::python::tuple rgb, const std::string &layer) {
	if(pos == nullptr)
		throw NullArgumentException("pos");
	self->drawText(QString::fromStdString(text), *pos, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_polygon(SystemState *self, boost::python::list points, boost::python::tuple rgb, const std::string &layer) {
	std::vector<Geometry2d::Point> ptVec;
	for (int i = 0; i < len(points); i++) {
		ptVec.push_back(boost::python::extract<Geometry2d::Point>(points[i]));
	}

	self->drawPolygon(ptVec, Color_from_tuple(rgb), QString::fromStdString(layer));
}

void State_draw_raw_polygon(SystemState *self, Geometry2d::Polygon points, boost::python::tuple rgb, const std::string &layer) {
    self->drawPolygon(points, Color_from_tuple(rgb), QString::fromStdString(layer));
}

boost::python::list Circle_intersects_line(Geometry2d::Circle *self, const Geometry2d::Line *line) {
	if(line == nullptr)
		throw NullArgumentException("line");
	boost::python::list lst;

	Geometry2d::Point intersectionPoints[2];
	int numIntersects = self->intersects(*line, intersectionPoints);
	for(int i = 0; i < numIntersects; i++) {
		lst.append(intersectionPoints[i]);
	}

	return lst;
}

boost::python::list Arc_intersects_line(Geometry2d::Arc *self, const Geometry2d::Line *line) {
	if(line == nullptr)
		throw NullArgumentException{"line"};
	boost::python::list lst;

	auto intersections = self->intersects(*line);

	for(auto& intersection : intersections) {
		lst.append(intersection);
	}

	return lst;
}

boost::python::list Arc_intersects_segment(Geometry2d::Arc *self, const Geometry2d::Segment *segment) {
	if(segment == nullptr)
		throw NullArgumentException{"segment"};
	boost::python::list lst;

	auto intersections = self->intersects(*segment);

	for(auto& intersection : intersections) {
		lst.append(intersection);
	}

	return lst;
}

Geometry2d::Point Circle_get_center(Geometry2d::Circle *self) {
    return self->center;
}

boost::python::tuple WinEval_eval_pt_to_seg(WindowEvaluator *self, const Geometry2d::Point *origin, const Geometry2d::Segment *target) {
	if(origin == nullptr)
		throw NullArgumentException{"origin"};
	if(target == nullptr)
		throw NullArgumentException{"target"};
	boost::python::list lst;

	auto window_results = self->eval_pt_to_seg(*origin, *target);

	lst.append(window_results.first);
	if(window_results.second.is_initialized())
		lst.append(window_results.second.get());
	else
		lst.append(boost::python::api::object());

	return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_pt(WindowEvaluator *self, const Geometry2d::Point *origin, const Geometry2d::Point *target) {
	if(origin == nullptr)
		throw NullArgumentException{"origin"};
	if(target == nullptr)
		throw NullArgumentException{"target"};
	boost::python::list lst;

	auto window_results = self->eval_pt_to_pt(*origin, *target);

	lst.append(window_results.first);
	if(window_results.second.is_initialized())
		lst.append(window_results.second.get());
	else
		lst.append(boost::python::api::object());

	return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_opp_goal(WindowEvaluator *self, const Geometry2d::Point *origin) {
	if(origin == nullptr)
		throw NullArgumentException{"origin"};
	boost::python::list lst;

	auto window_results = self->eval_pt_to_opp_goal(*origin);

	lst.append(window_results.first);
	if(window_results.second.is_initialized())
		lst.append(window_results.second.get());
	else
		lst.append(boost::python::api::object());

	return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_our_goal(WindowEvaluator *self, const Geometry2d::Point *origin) {
	if(origin == nullptr)
		throw NullArgumentException{"origin"};
	boost::python::list lst;

	auto window_results = self->eval_pt_to_our_goal(*origin);

	lst.append(window_results.first);
	if(window_results.second.is_initialized())
		lst.append(window_results.second.get());
	else
		lst.append(boost::python::api::object());

	return boost::python::tuple{lst};
}

void WinEval_add_excluded_robot(WindowEvaluator *self, Robot* robot) {
	self->excluded_robots.push_back(robot);
}

/**
 * The code in this block wraps up c++ classes and makes them
 * accessible to python in the 'robocup' module.
 */
BOOST_PYTHON_MODULE(robocup)
{
	boost::python::register_exception_translator<NullArgumentException>(&translateException);

	def("fix_angle_radians", &fixAngleRadians);
	def("get_trapezoidal_time", &Trapezoidal::getTime);

	class_<Geometry2d::Point, Geometry2d::Point*>("Point", init<float, float>())
		.def(init<const Geometry2d::Point &>())
		.def_readwrite("x", &Geometry2d::Point::x)
		.def_readwrite("y", &Geometry2d::Point::y)
		.def(self - self)
		.def(self + self)
		.def("mag", &Geometry2d::Point::mag)
		.def("magsq", &Geometry2d::Point::magsq)
		.def("__repr__", &Point_repr)
		.def("normalized", &Geometry2d::Point::normalized)
		.def("rotate", &Point_rotate)
		.def(self * float())
		.def(self / float())
		.def("perp_ccw", &Geometry2d::Point::perpCCW)
		.def("perp_cw", &Geometry2d::Point::perpCW)
		.def("angle", &Geometry2d::Point::angle)
		.def("dot", &Geometry2d::Point::dot)
		.def("near_point", &Geometry2d::Point::nearPoint)
		.def("dist_to", &Geometry2d::Point::distTo)
		.def("direction", &Geometry2d::Point::direction)
		.staticmethod("direction")
	;

	class_<Geometry2d::Line, Geometry2d::Line*>("Line", init<Geometry2d::Point, Geometry2d::Point>())
		.def("delta", &Geometry2d::Line::delta)
		.def("line_intersection", &Line_line_intersection)
		.def("dist_to", &Geometry2d::Line::distTo)
		.def("intersects_circle", &Line_intersects_circle)
		.def("get_pt", &Line_get_pt, return_value_policy<reference_existing_object>())
		.def("nearest_point", &Geometry2d::Line::nearestPoint)
	;

	class_<Geometry2d::Segment, Geometry2d::Segment*, bases<Geometry2d::Line> >("Segment", init<Geometry2d::Point, Geometry2d::Point>())
		.def("center", &Geometry2d::Segment::center)
		.def("length", &Geometry2d::Segment::length)
		.def("dist_to", &Geometry2d::Segment::distTo)
		.def("nearest_point_to_point", &Segment_nearest_point_to_point)
		.def("segment_intersection", &Segment_segment_intersection)
		.def("line_intersection", &Segment_line_intersection)
		.def("near_point", &Geometry2d::Segment::nearPoint)
		.def("nearest_point_to_line", &Segment_nearest_point_to_line)
		.def("__str__", &Geometry2d::Segment::toString)
	;

	class_<Geometry2d::Shape, boost::noncopyable>("Shape")
	;

	class_<Geometry2d::Rect, bases<Geometry2d::Shape> >("Rect", init<Geometry2d::Point, Geometry2d::Point>())
		.def("contains_rect", &Rect_contains_rect)
		.def("min_x", &Geometry2d::Rect::minx)
		.def("min_y", &Geometry2d::Rect::miny)
		.def("max_x", &Geometry2d::Rect::maxx)
		.def("max_y", &Geometry2d::Rect::maxy)
		.def("near_point", &Geometry2d::Rect::nearPoint)
		.def("intersects_rect", &Geometry2d::Rect::intersects)
		.def("contains_point", &Geometry2d::Rect::containsPoint)
		.def("get_pt", &Rect_get_pt, return_value_policy<reference_existing_object>())
	;

	class_<Geometry2d::Circle, bases<Geometry2d::Shape> >("Circle", init<Geometry2d::Point, float>())
		.def("intersects_line", &Circle_intersects_line)
		.def("nearest_point", &Geometry2d::Circle::nearestPoint)
        .def("contains_point", &Geometry2d::Circle::containsPoint)
        .def("center", &Circle_get_center)
	;

	class_<Geometry2d::Arc>("Arc", init<Geometry2d::Point, float, float, float>())
		.def("intersects_line", &Arc_intersects_line)
		.def("intersects_segment", &Arc_intersects_segment)
		.def("center", &Geometry2d::Arc::center)
		.def("radius", &Geometry2d::Arc::radius)
		.def("start", &Geometry2d::Arc::start)
		.def("end", &Geometry2d::Arc::end)
	;

	class_<Geometry2d::Arc>("Arc", init<Geometry2d::Point, float, float, float>())
		.def("intersects_line", &Arc_intersects_line)
		.def("intersects_segment", &Arc_intersects_segment)
		.def("center", &Geometry2d::Arc::center)
		.def("radius", &Geometry2d::Arc::radius)
		.def("start", &Geometry2d::Arc::start)
		.def("end", &Geometry2d::Arc::end)
	;

	class_<Geometry2d::CompositeShape, bases<Geometry2d::Shape> >("CompositeShape", init<>())
		.def("clear", &Geometry2d::CompositeShape::clear)
		.def("is_empty", &Geometry2d::CompositeShape::empty)
		.def("size", &Geometry2d::CompositeShape::size)
		.def("add_shape", &CompositeShape_add_shape)
		.def("contains_point", &Geometry2d::CompositeShape::containsPoint)
	;

	class_<Geometry2d::Polygon, bases<Geometry2d::Shape> >("Polygon", init<>())
		.def("add_vertex", &Polygon_add_vertex)
        .def("contains_point", &Geometry2d::Polygon::containsPoint)
	;

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
		.def("is_setup_state", &GameState::inSetupState)
		.def("is_ready_state", &GameState::inReadyState)
		.def("can_kick", &GameState::canKick)
		.def("stay_away_from_ball", &GameState::stayAwayFromBall)
		.def("stay_on_side", &GameState::stayOnSide)
		.def("stay_behind_penalty_line", &GameState::stayBehindPenaltyLine)
		.def("is_our_restart", &GameState::isOurRestart)
	;

	class_<Robot>("Robot", init<int, bool>())
		.def("shell_id", &Robot::shell)
		.def("is_ours", &Robot::self, "whether or not this robot is on our team")
		.add_property("pos", &Robot_pos, "position vector of the robot in meters")
		.add_property("vel", &Robot_vel, "velocity vector of the robot in m/s")
		.add_property("angle", &Robot_angle, "angle of the robot in degrees")
		.add_property("angle_vel", &Robot_angle_vel, "angular velocity in degrees per second")
        .add_property("visible", &Robot::visible)
		.def("__repr__", &Robot_repr)
		.def("__eq__", &Robot::equals)
	;

	class_<OurRobot, OurRobot *, bases<Robot>, boost::noncopyable>("OurRobot", init<int, SystemState*>())
		.def("move_to", &OurRobot_move_to)
		.def("move_to_end_vel", &OurRobot_move_to_end_vel)
		.def("move_to_direct", &OurRobot_move_to_direct)
		.def("set_world_vel", &OurRobot::worldVelocity)
		.def("set_angle_vel", &OurRobot::angleVelocity)
		.def("face", &OurRobot::face)
		.def("pivot", &OurRobot::pivot)
		.def("set_max_angle_speed", OurRobot_set_max_angle_speed)
		.def("set_avoid_ball_radius", &OurRobot_set_avoid_ball_radius)
		.def("shield_from_teammates", &OurRobot::shieldFromTeammates)
		.def("set_avoid_teammate_radius", OurRobot_set_avoid_teammate_radius)
		.def("disable_avoid_ball", &OurRobot::disableAvoidBall)
		.def("avoid_all_teammates", &OurRobot::avoidAllTeammates)
		.def("add_text", &OurRobot_add_text)
		.def("approach_opponent", &OurRobot_approach_opponent)
		.def("set_avoid_opponents", &OurRobot_set_avoid_opponents)
		.def("set_dribble_speed", &OurRobot::dribble)
		.def("has_ball", &OurRobot::hasBall)
		.def("last_kick_time", &OurRobot::lastKickTime)
		.def("just_kicked", &OurRobot::justKicked)
		.def("has_chipper", &OurRobot::chipper_available)
		.def("face_none", &OurRobot::faceNone)
		.def("kick", &OurRobot::kick)
		.def("kick_level", &OurRobot::kickLevel)
		.def("chip", &OurRobot::chip)
		.def("chip_level", &OurRobot::chipLevel)
		.def("unkick", &OurRobot::unkick, "clears any prevous kick command sent to the robot")
		.def("get_cmd_text", &OurRobot::getCmdText, "gets the string containing a list of commands sent to the robot, such as face(), move_to(), etc.")
		.def("ball_sense_works", &OurRobot::ballSenseWorks)
		.def("kicker_works", &OurRobot::kickerWorks)
		.def("add_local_obstacle", &OurRobot_add_local_obstacle)
		.def_readwrite("is_penalty_kicker", &OurRobot::isPenaltyKicker)
	;

	class_<OpponentRobot, OpponentRobot *, std::shared_ptr<OpponentRobot>, bases<Robot> >("OpponentRobot", init<int>());

	class_<Ball, std::shared_ptr<Ball> >("Ball", init<>())
		.def_readonly("pos", &Ball::pos)
		.def_readonly("vel", &Ball::vel)
		.def_readonly("valid", &Ball::valid)
	;

	class_<std::vector<Robot*>>("vector_Robot")
		.def(vector_indexing_suite<std::vector<Robot*>>())
		.def("clear", &std::vector<Robot*>::clear)
	;

	class_<std::vector<OurRobot *> >("vector_OurRobot")
		.def(vector_indexing_suite<std::vector<OurRobot *> >())
	;

	class_<std::vector<OpponentRobot *> >("vector_OpponentRobot")
		.def(vector_indexing_suite<std::vector<OpponentRobot *> >())
	;

	class_<SystemState, SystemState *>("SystemState")
		.def_readonly("our_robots", &SystemState::self)
		.def_readonly("their_robots", &SystemState::opp)
		.def_readonly("ball", &SystemState::ball)
		.def_readonly("game_state", &SystemState::gameState)
		.def_readonly("timestamp", &SystemState::timestamp)

		//	debug drawing methods
		.def("draw_circle", &State_draw_circle)
		.def("draw_text", &State_draw_text)
		.def("draw_shape", &SystemState::drawShape)
		.def("draw_line", &State_draw_line)
		.def("draw_segment", &State_draw_segment)
		.def("draw_polygon", &State_draw_polygon)
		.def("draw_arc", &State_draw_arc)
        .def("draw_raw_polygon", &State_draw_raw_polygon)
		.def("draw_arc", &State_draw_arc)
	;

	class_<Field_Dimensions>("Field_Dimensions")
		.def("Length", &Field_Dimensions::Length)
		.def("Width", &Field_Dimensions::Width)
		.def("Border", &Field_Dimensions::Border)
		.def("LineWidth", &Field_Dimensions::LineWidth)
		.def("GoalWidth", &Field_Dimensions::GoalWidth)
		.def("GoalDepth", &Field_Dimensions::GoalDepth)
		.def("GoalHeight", &Field_Dimensions::GoalHeight)
		.def("PenaltyDist", &Field_Dimensions::PenaltyDist)
		.def("PenaltyDiam", &Field_Dimensions::PenaltyDiam)
		.def("ArcRadius", &Field_Dimensions::ArcRadius)
		.def("CenterRadius", &Field_Dimensions::CenterRadius)
		.def("CenterDiameter", &Field_Dimensions::CenterDiameter)
		.def("GoalFlat", &Field_Dimensions::GoalFlat)
		.def("FloorLength", &Field_Dimensions::FloorLength)
		.def("FloorWidth", &Field_Dimensions::FloorWidth)
	;

	class_<Window>("Window")
		.def_readwrite("a0", &Window::a0)
		.def_readwrite("a1", &Window::a1)
		.def_readwrite("t0", &Window::t0)
		.def_readwrite("t1", &Window::t1)
		.def_readwrite("segment", &Window::segment)
		.def_readwrite("shot_success", &Window::shot_success)
	;

	class_<std::vector<Window>>("vector_Window")
		.def(vector_indexing_suite<std::vector<Window>>())
	;

	class_<WindowEvaluator>("WindowEvaluator", init<SystemState*>())
		.def_readwrite("debug", &WindowEvaluator::debug)
		.def_readwrite("chip_enabled", &WindowEvaluator::chip_enabled)
		.def_readwrite("max_chip_range", &WindowEvaluator::max_chip_range)
		.def_readwrite("min_chip_range", &WindowEvaluator::min_chip_range)
		.def_readwrite("excluded_robots", &WindowEvaluator::excluded_robots)
		.def_readwrite("hypothetical_robot_locations", &WindowEvaluator::hypothetical_robot_locations)
		.def("add_excluded_robot", &WinEval_add_excluded_robot)
		.def("eval_pt_to_pt", &WinEval_eval_pt_to_pt)
		.def("eval_pt_to_opp_goal", &WinEval_eval_pt_to_opp_goal)
		.def("eval_pt_to_our_goal", &WinEval_eval_pt_to_our_goal)
		.def("eval_pt_to_seg", &WinEval_eval_pt_to_seg)
		.def("obstacle_range", &WindowEvaluator::obstacle_range)
		.def("obstacle_robot", &WindowEvaluator::obstacle_robot)
	;

}
