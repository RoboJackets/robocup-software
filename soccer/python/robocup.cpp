#include <boost/python.hpp>
using namespace boost::python;

#include <Geometry2d/Point.hpp>
#include <Robot.hpp>



BOOST_PYTHON_MODULE(robocup)
{
	class_<Geometry2d::Point>("Point", init<float, float>())
		.def_readwrite("x", &Geometry2d::Point::x)
		.def_readwrite("y", &Geometry2d::Point::y)
	;


	class_<Robot>("Robot", init<int, bool>())
		.def("shell_id", &Robot::shell)
		.def("is_ours", &Robot::self)
		.def_readwrite("pos", &Robot::pos)
		.def_readwrite("vel", &Robot::vel)
		.def_readwrite("angle", &Robot::angle)
		.def_readwrite("angle_vel", &Robot::angleVel)
	;
}
