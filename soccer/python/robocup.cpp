
#include <boost/python.hpp>
using namespace boost::python;


BOOST_PYTHON_MODULE(robocup)
{
	class_<Point>("Point", init<float, float>())
		.add_property("x", &Point::x, &Point::set_x)
		.add_property("y", &Point::y, &Point::set_y)
	;

	class_<Robot>("Robot", init<>())
		.def_readwrite("pos", &Robot::pos)
		.def_readwrite("vel", &Robot::vel)
		.def_readwrite("angle", &Robot::angle)
		.def_readwrite("angle_vel", &Robot::angle_vel)
	;
}
