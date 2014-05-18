
#include <boost/python.hpp>
using namespace boost::python;


class Point {
public:
	Point(float x = 0, float y = 0) {
		_x = x;
		_y = y;
	}

	float x() const {
		return _x;
	}
	void set_x(float x) {
		_x = x;
	}

	float y() const {
		return _y;
	}
	void set_y(float y) {
		_y = y;
	}

private:
	float _x, _y;
};


class Robot {
public:
	Point pos, vel;
	float angle, angle_vel;
};


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


