#include "robocup-py.hpp"
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/register_ptr_to_python.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <functional>

using namespace boost::python;

#include "motion/TrapezoidalMotion.hpp"
#include "planning/MotionConstraints.hpp"
#include "KickEvaluator.hpp"
#include "WindowEvaluator.hpp"
#include "optimization/NelderMead2D.hpp"
#include "optimization/NelderMead2DConfig.hpp"
#include <Constants.hpp>
#include <Geometry2d/Arc.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Rect.hpp>
#include <Geometry2d/Line.hpp>
#include <protobuf/LogFrame.pb.h>
#include <Robot.hpp>
#include <motion/MotionControl.hpp>
#include <Pid.hpp>
#include <SystemState.hpp>

#include <boost/python/exception_translator.hpp>
#include <boost/version.hpp>
#include <exception>

#include "RobotConfig.hpp"
#include <Configuration.hpp>

/**
 * These functions make sure errors on the c++
 * side get passed up through python.
 */

struct NullArgumentException : public std::exception {
    std::string argument_name;
    NullArgumentException() : argument_name("") {}
    NullArgumentException(std::string name) : argument_name(name) {}
    virtual const char* what() const throw() override {
        return ("'" + argument_name + "'' was 'None'.").c_str();
    }
};

void translateException(NullArgumentException const& e) {
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

/**
 * NOTES FOR WRAPPER FUNCTIONS/METHODS
 *
 * Keep in mind that pointer parameters will be be nullptr/NULL if the value
 * from python was None.  Check for this case so that we don't segfault.
 */

// this is here so boost can work with std::shared_ptr
// later versions of boost include this, so we only define it for older
// versions
#if BOOST_VERSION < 105300
template <class T>
T* get_pointer(std::shared_ptr<T> const& p) {
    return p.get();
}
#endif

QColor Color_from_tuple(const boost::python::tuple& rgb) {
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

std::string Point_repr(Geometry2d::Point* self) { return self->toString(); }

std::string Robot_repr(Robot* self) { return self->toString(); }

Geometry2d::Point Robot_pos(Robot* self) { return self->pos; }

// Sets a robot's position - this should never be used in gameplay code, but
// is useful for testing.
void Robot_set_pos_for_testing(Robot* self, Geometry2d::Point pos) {
    self->pos = pos;
}

Geometry2d::Point Robot_vel(Robot* self) { return self->vel; }

float Robot_angle(Robot* self) { return self->angle; }

float Robot_angle_vel(Robot* self) { return self->angleVel; }

void OurRobot_move_to_direct(OurRobot* self, Geometry2d::Point* to) {
    if (to == nullptr) throw NullArgumentException("to");
    self->moveDirect(*to);
}

void OurRobot_move_tuning(OurRobot* self, Geometry2d::Point* to) {
    if (to == nullptr) throw NullArgumentException("to");
    self->moveTuning(*to);
}

void OurRobot_move_to_end_vel(OurRobot* self, Geometry2d::Point* endPos,
                              Geometry2d::Point* vf) {
    if (endPos == nullptr || vf == nullptr) throw NullArgumentException();
    self->move(*endPos, *vf);
}

void OurRobot_move_to(OurRobot* self, Geometry2d::Point* to) {
    if (to == nullptr) throw NullArgumentException("to");
    self->move(*to);
}

void OurRobot_add_local_obstacle(OurRobot* self, Geometry2d::Shape* obs) {
    if (obs == nullptr) throw NullArgumentException("obs");
    std::shared_ptr<Geometry2d::Shape> sharedObs(obs->clone());
    self->localObstacles(sharedObs);
}

void OurRobot_set_avoid_ball_radius(OurRobot* self, float radius) {
    self->avoidBallRadius(radius);
}

void OurRobot_set_max_angle_speed(OurRobot* self, float maxAngleSpeed) {
    self->rotationConstraints().maxSpeed = maxAngleSpeed;
}

void OurRobot_set_max_speed(OurRobot* self, float maxSpeed) {
    self->motionConstraints().maxSpeed = maxSpeed;
}

void OurRobot_approach_opponent(OurRobot* self, unsigned shell_id,
                                bool enable_approach) {
    self->approachOpponent(shell_id, enable_approach);
}

void OurRobot_add_text(OurRobot* self, const std::string& text,
                       boost::python::tuple rgb,
                       const std::string& layerPrefix) {
    self->addText(QString::fromStdString(text), Color_from_tuple(rgb),
                  QString::fromStdString(layerPrefix));
}

void OurRobot_set_avoid_opponents(OurRobot* self, bool value) {
    self->avoidOpponents(value);
}

void OurRobot_initialize_tuner(OurRobot* self, char controller) {
    self->motionControl()->getPid(controller)->initializeTuner();
}

void OurRobot_start_pid_tuner(OurRobot* self, char controller) {
    self->motionControl()->getPid(controller)->startTunerCycle();
    self->config->translation.p->setValue(
        self->motionControl()->getPid(controller)->kp);
    self->config->translation.i->setValue(
        self->motionControl()->getPid(controller)->ki);
    self->config->translation.d->setValue(
        self->motionControl()->getPid(controller)->kd);
}

void OurRobot_run_pid_tuner(OurRobot* self, char controller) {
    self->motionControl()->getPid(controller)->runTuner();
}

bool OurRobot_end_pid_tuner(OurRobot* self, char controller) {
    return self->motionControl()->getPid(controller)->endTunerCycle();
}

bool Rect_contains_rect(Geometry2d::Rect* self, Geometry2d::Rect* other) {
    if (other == nullptr) throw NullArgumentException("other");
    return self->containsRect(*other);
}

bool Rect_contains_point(Geometry2d::Rect* self, Geometry2d::Point* pt) {
    if (pt == nullptr) throw NullArgumentException("pt");
    return self->containsPoint(*pt);
}

void Point_rotate(Geometry2d::Point* self, Geometry2d::Point* origin,
                  float angle) {
    if (origin == nullptr) throw NullArgumentException("origin");
    self->rotate(*origin, angle);
}

void CompositeShape_add_shape(Geometry2d::CompositeShape* self,
                              Geometry2d::Shape* shape) {
    if (shape == nullptr) throw NullArgumentException("shape");
    self->add(std::shared_ptr<Geometry2d::Shape>(shape->clone()));
}

void Polygon_add_vertex(Geometry2d::Polygon* self, Geometry2d::Point* pt) {
    if (pt == nullptr) throw NullArgumentException("pt");
    self->addVertex(*pt);
}

Geometry2d::Point* Line_get_pt(Geometry2d::Line* self, int index) {
    return &(self->pt[index]);
}

Geometry2d::Point* Segment_get_pt(Geometry2d::Segment* self, int index) {
    return &(self->pt[index]);
}

Geometry2d::Point* Rect_get_pt(Geometry2d::Rect* self, int index) {
    return &(self->pt[index]);
}

boost::python::object Segment_segment_intersection(Geometry2d::Segment* self,
                                                   Geometry2d::Segment* other) {
    if (other == nullptr) throw NullArgumentException("other");
    Geometry2d::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
}

boost::python::object Segment_line_intersection(Geometry2d::Segment* self,
                                                Geometry2d::Line* line) {
    if (line == nullptr) throw NullArgumentException{"line"};
    Geometry2d::Point pt;
    if (self->intersects(*line, &pt)) {
        boost::python::object obj{pt};
        return obj;
    } else {
        return boost::python::object{};
    }
}

boost::python::object Segment_nearest_point_to_point(Geometry2d::Segment* self,
                                                     Geometry2d::Point* point) {
    if (point == nullptr) throw NullArgumentException{"point"};
    return boost::python::object{self->nearestPoint(*point)};
}

boost::python::object Segment_nearest_point_to_line(Geometry2d::Segment* self,
                                                    Geometry2d::Line* line) {
    if (line == nullptr) throw NullArgumentException{"line"};
    return boost::python::object{self->nearestPoint(*line)};
}

// returns None or a Geometry2d::Point
boost::python::object Line_line_intersection(Geometry2d::Line* self,
                                             Geometry2d::Line* other) {
    if (other == nullptr) throw NullArgumentException("other");
    Geometry2d::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
};

boost::python::object Line_segment_intersection(Geometry2d::Line* self,
                                                Geometry2d::Segment* other) {
    if (other == nullptr) throw NullArgumentException("other");
    Geometry2d::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
};

boost::python::tuple Line_intersects_circle(Geometry2d::Line* self,
                                            Geometry2d::Circle* circle) {
    if (circle == nullptr) throw NullArgumentException("circle");
    Geometry2d::Point a, b;
    boost::python::list lst;

    bool intersects = self->intersects(*circle, &a, &b);
    lst.append(intersects);
    lst.append(a);
    lst.append(b);

    return boost::python::tuple(lst);
}

void State_draw_circle(SystemState* self, const Geometry2d::Point* center,
                       float radius, boost::python::tuple rgb,
                       const std::string& layer) {
    if (center == nullptr) throw NullArgumentException("center");
    self->drawCircle(*center, radius, Color_from_tuple(rgb),
                     QString::fromStdString(layer));
}

void State_draw_arc(SystemState* self, const Geometry2d::Arc* arc,
                    boost::python::tuple rgb, const std::string& layer) {
    if (arc == nullptr) throw NullArgumentException{"arc"};
    self->drawArc(*arc, Color_from_tuple(rgb), QString::fromStdString(layer));
}

// TODO(ashaw596) Fix this lie of a function
void State_draw_line(SystemState* self, const Geometry2d::Line* line,
                     boost::python::tuple rgb, const std::string& layer) {
    if (line == nullptr) throw NullArgumentException("line");
    self->drawLine(Geometry2d::Segment(*line), Color_from_tuple(rgb),
                   QString::fromStdString(layer));
}

void State_draw_segment(SystemState* self, const Geometry2d::Segment* segment,
                        boost::python::tuple rgb, const std::string& layer) {
    if (segment == nullptr) throw NullArgumentException("segment");
    self->drawSegment(*segment, Color_from_tuple(rgb),
                      QString::fromStdString(layer));
}

void State_draw_segment_from_points(SystemState* self,
                                    const Geometry2d::Point* p0,
                                    const Geometry2d::Point* p1,
                                    boost::python::tuple rgb,
                                    const std::string& layer) {
    if (p0 == nullptr) throw NullArgumentException{"p0"};
    if (p1 == nullptr) throw NullArgumentException{"p1"};
    self->drawLine(*p0, *p1, Color_from_tuple(rgb),
                   QString::fromStdString(layer));
}

void State_draw_text(SystemState* self, const std::string& text,
                     Geometry2d::Point* pos, boost::python::tuple rgb,
                     const std::string& layer) {
    if (pos == nullptr) throw NullArgumentException("pos");
    self->drawText(QString::fromStdString(text), *pos, Color_from_tuple(rgb),
                   QString::fromStdString(layer));
}

void State_draw_polygon(SystemState* self, const boost::python::list& points,
                        boost::python::tuple rgb, const std::string& layer) {
    std::vector<Geometry2d::Point> ptVec;
    for (int i = 0; i < len(points); i++) {
        ptVec.push_back(boost::python::extract<Geometry2d::Point>(points[i]));
    }

    self->drawPolygon(ptVec, Color_from_tuple(rgb),
                      QString::fromStdString(layer));
}

void State_draw_raw_polygon(SystemState* self, Geometry2d::Polygon points,
                            boost::python::tuple rgb,
                            const std::string& layer) {
    self->drawPolygon(points, Color_from_tuple(rgb),
                      QString::fromStdString(layer));
}

boost::python::list Circle_intersects_line(Geometry2d::Circle* self,
                                           const Geometry2d::Line* line) {
    if (line == nullptr) throw NullArgumentException("line");
    boost::python::list lst;

    Geometry2d::Point intersectionPoints[2];
    int numIntersects = self->intersects(*line, intersectionPoints);
    for (int i = 0; i < numIntersects; i++) {
        lst.append(intersectionPoints[i]);
    }

    return lst;
}

boost::python::list Arc_intersects_line(Geometry2d::Arc* self,
                                        const Geometry2d::Line* line) {
    if (line == nullptr) throw NullArgumentException{"line"};
    boost::python::list lst;

    auto intersections = self->intersects(*line);

    for (auto& intersection : intersections) {
        lst.append(intersection);
    }

    return lst;
}

boost::python::list Arc_intersects_segment(Geometry2d::Arc* self,
                                           const Geometry2d::Segment* segment) {
    if (segment == nullptr) throw NullArgumentException{"segment"};
    boost::python::list lst;

    auto intersections = self->intersects(*segment);

    for (auto& intersection : intersections) {
        lst.append(intersection);
    }

    return lst;
}

boost::python::tuple WinEval_eval_pt_to_seg(WindowEvaluator* self,
                                            const Geometry2d::Point* origin,
                                            const Geometry2d::Segment* target) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto window_results = self->eval_pt_to_seg(*origin, *target);

    lst.append(window_results.first);
    if (window_results.second.is_initialized())
        lst.append(window_results.second.get());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_robot(WindowEvaluator* self,
                                              const Geometry2d::Point* origin,
                                              const Geometry2d::Point* target) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto window_results = self->eval_pt_to_robot(*origin, *target);

    lst.append(window_results.first);
    if (window_results.second.is_initialized())
        lst.append(window_results.second.get());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_pt(WindowEvaluator* self,
                                           const Geometry2d::Point* origin,
                                           const Geometry2d::Point* target,
                                           float targetWidth) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto window_results = self->eval_pt_to_pt(*origin, *target, targetWidth);

    lst.append(window_results.first);
    if (window_results.second.is_initialized())
        lst.append(window_results.second.get());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_opp_goal(
    WindowEvaluator* self, const Geometry2d::Point* origin) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    boost::python::list lst;

    auto window_results = self->eval_pt_to_opp_goal(*origin);

    lst.append(window_results.first);
    if (window_results.second.is_initialized())
        lst.append(window_results.second.get());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple WinEval_eval_pt_to_our_goal(
    WindowEvaluator* self, const Geometry2d::Point* origin) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    boost::python::list lst;

    auto window_results = self->eval_pt_to_our_goal(*origin);

    lst.append(window_results.first);
    if (window_results.second.is_initialized())
        lst.append(window_results.second.get());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

void WinEval_add_excluded_robot(WindowEvaluator* self, Robot* robot) {
    self->excluded_robots.push_back(robot);
}

boost::python::tuple KickEval_eval_pt_to_seg(
    KickEvaluator* self, const Geometry2d::Point* origin,
    const Geometry2d::Segment* target) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_seg(*origin, *target);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple KickEval_eval_pt_to_robot(
    KickEvaluator* self, const Geometry2d::Point* origin,
    const Geometry2d::Point* target) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_robot(*origin, *target);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple KickEval_eval_pt_to_pt(KickEvaluator* self,
                                            const Geometry2d::Point* origin,
                                            const Geometry2d::Point* target,
                                            float targetWidth) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    if (target == nullptr) throw NullArgumentException{"target"};
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_pt(*origin, *target, targetWidth);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple KickEval_eval_pt_to_opp_goal(
    KickEvaluator* self, const Geometry2d::Point* origin) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_opp_goal(*origin);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple KickEval_eval_pt_to_our_goal(
    KickEvaluator* self, const Geometry2d::Point* origin) {
    if (origin == nullptr) throw NullArgumentException{"origin"};
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_our_goal(*origin);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

void KickEval_add_excluded_robot(KickEvaluator* self, Robot* robot) {
    self->excluded_robots.push_back(robot);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Point_overloads, normalized, 0, 1)

float Point_get_x(const Geometry2d::Point* self) { return self->x(); }
float Point_get_y(const Geometry2d::Point* self) { return self->y(); }
void Point_set_x(Geometry2d::Point* self, float x) { self->x() = x; }
void Point_set_y(Geometry2d::Point* self, float y) { self->y() = y; }

/**
 * Python function must be in the form...
 * [float] pythonFunc(... float x, float y)
 */
float point_python_callback(Geometry2d::Point p, PyObject* pyfun) {
    PyObject* pyresult =
        PyObject_CallObject(pyfun, Py_BuildValue("ff", p.x(), p.y()));

    if (pyresult == NULL) {
        std::cerr << "Python callback function returned a bad value with args ";
        std::cerr << p << std::endl;
        return -1;
    }

    return PyFloat_AsDouble(pyresult);
}

boost::shared_ptr<std::function<float(Geometry2d::Point)>>
stdfunction_constructor(PyObject* function) {
    Py_INCREF(function);

    // Create aliased function to hid python function args
    std::function<float(Geometry2d::Point)> f =
        std::bind(&point_python_callback, std::placeholders::_1, function);

    return boost::shared_ptr<std::function<float(Geometry2d::Point)>>(
        new std::function<float(Geometry2d::Point)>(f));
}

boost::shared_ptr<NelderMead2DConfig> NelderMead2DConfig_constructor(
    std::function<float(Geometry2d::Point)>* function,
    Geometry2d::Point start = Geometry2d::Point(0, 0),
    Geometry2d::Point step = Geometry2d::Point(1, 1),
    Geometry2d::Point minDist = Geometry2d::Point(0.001, 0.001),
    float reflectionCoeff = 1, float expansionCoeff = 2,
    float contractionCoeff = 0.5, float shrinkCoeff = 0.5,
    int maxIterations = 100, float maxValue = 0, float maxThresh = 0) {
    return boost::shared_ptr<NelderMead2DConfig>(new NelderMead2DConfig(
        *function, start, step, minDist, reflectionCoeff, expansionCoeff,
        contractionCoeff, shrinkCoeff, maxIterations, maxValue, maxThresh));
}

boost::shared_ptr<NelderMead2D> NelderMead2D_constructor(
    NelderMead2DConfig* config) {
    return boost::shared_ptr<NelderMead2D>(new NelderMead2D(*config));
}

/**
 * The code in this block wraps up c++ classes and makes them
 * accessible to python in the 'robocup' module.
 */
BOOST_PYTHON_MODULE(robocup) {
    boost::python::register_exception_translator<NullArgumentException>(
        &translateException);

    def("fix_angle_radians", &fixAngleRadians);
    def("get_trapezoidal_time", &Trapezoidal::getTime);

    class_<Geometry2d::Point, Geometry2d::Point*>("Point", init<float, float>())
        .def(init<const Geometry2d::Point&>())
        .add_property("x", &Point_get_x, &Point_set_x)
        .add_property("y", &Point_get_y, &Point_set_y)
        .def(self - self)
        .def(self + self)
        .def("mag", &Geometry2d::Point::mag)
        .def("magsq", &Geometry2d::Point::magsq)
        .def("__repr__", &Point_repr)
        .def("normalized", &Geometry2d::Point::normalized, Point_overloads())
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
        .def("angle_between", &Geometry2d::Point::angleBetween)
        .def("nearly_equals", &Geometry2d::Point::nearlyEquals)
        .staticmethod("direction");

    class_<Geometry2d::Line, Geometry2d::Line*>(
        "Line", init<Geometry2d::Point, Geometry2d::Point>())
        .def("delta", &Geometry2d::Line::delta)
        .def("line_intersection", &Line_line_intersection)
        .def("segment_intersection", &Line_segment_intersection)
        .def("dist_to", &Geometry2d::Line::distTo)
        .def("intersects_circle", &Line_intersects_circle)
        .def("get_pt", &Line_get_pt,
             return_value_policy<reference_existing_object>())
        .def("nearest_point", &Geometry2d::Line::nearestPoint);

    class_<Geometry2d::Segment, Geometry2d::Segment*>(
        "Segment", init<Geometry2d::Point, Geometry2d::Point>())
        .def("center", &Geometry2d::Segment::center)
        .def("length", &Geometry2d::Segment::length)
        .def("dist_to", &Geometry2d::Segment::distTo)
        .def("get_pt", &Segment_get_pt,
             return_value_policy<reference_existing_object>())
        .def("nearest_point", &Segment_nearest_point_to_point)
        .def("segment_intersection", &Segment_segment_intersection)
        .def("line_intersection", &Segment_line_intersection)
        .def("near_point", &Geometry2d::Segment::nearPoint)
        .def("nearest_point_to_line", &Segment_nearest_point_to_line)
        .def("__str__", &Geometry2d::Segment::toString);

    class_<Geometry2d::Shape, boost::noncopyable>("Shape");

    class_<Geometry2d::Rect, bases<Geometry2d::Shape>>(
        "Rect", init<Geometry2d::Point, Geometry2d::Point>())
        .def("contains_rect", &Rect_contains_rect)
        .def("min_x", &Geometry2d::Rect::minx)
        .def("min_y", &Geometry2d::Rect::miny)
        .def("max_x", &Geometry2d::Rect::maxx)
        .def("max_y", &Geometry2d::Rect::maxy)
        .def("near_point", &Geometry2d::Rect::nearPoint)
        .def("intersects_rect", &Geometry2d::Rect::intersects)
        .def("contains_point", &Geometry2d::Rect::containsPoint)
        .def("get_pt", &Rect_get_pt,
             return_value_policy<reference_existing_object>());

    class_<Geometry2d::Circle, bases<Geometry2d::Shape>>(
        "Circle", init<Geometry2d::Point, float>())
        .def("intersects_line", &Circle_intersects_line)
        .def("nearest_point", &Geometry2d::Circle::nearestPoint)
        .def("contains_point", &Geometry2d::Circle::containsPoint)
        .def_readonly("center", &Geometry2d::Circle::center);

    class_<Geometry2d::Arc>("Arc",
                            init<Geometry2d::Point, float, float, float>())
        .def("intersects_line", &Arc_intersects_line)
        .def("intersects_segment", &Arc_intersects_segment)
        .def("center", &Geometry2d::Arc::center)
        .def("radius", &Geometry2d::Arc::radius)
        .def("start", &Geometry2d::Arc::start)
        .def("end", &Geometry2d::Arc::end);

    class_<Geometry2d::CompositeShape, bases<Geometry2d::Shape>>(
        "CompositeShape", init<>())
        .def("clear", &Geometry2d::CompositeShape::clear)
        .def("is_empty", &Geometry2d::CompositeShape::empty)
        .def("size", &Geometry2d::CompositeShape::size)
        .def("add_shape", &CompositeShape_add_shape)
        .def("contains_point", &Geometry2d::CompositeShape::containsPoint);

    class_<Geometry2d::Polygon, bases<Geometry2d::Shape>>("Polygon", init<>())
        .def("add_vertex", &Polygon_add_vertex)
        .def("contains_point", &Geometry2d::Polygon::containsPoint);

    class_<GameState>("GameState")
        .def_readonly("our_score", &GameState::ourScore)
        .def_readonly("their_score", &GameState::theirScore)
        .def("is_halted", &GameState::halt)
        .def("is_stopped", &GameState::stopped)
        .def("is_playing", &GameState::playing)
        .def("is_kickoff", &GameState::kickoff)
        .def("is_penalty", &GameState::penalty)
        .def("is_placement", &GameState::placement)
        .def("is_direct", &GameState::direct)
        .def("is_indirect", &GameState::indirect)
        .def("is_our_kickoff", &GameState::ourKickoff)
        .def("is_our_penalty", &GameState::ourPenalty)
        .def("is_our_direct", &GameState::ourDirect)
        .def("is_our_indirect", &GameState::ourIndirect)
        .def("is_our_free_kick", &GameState::ourFreeKick)
        .def("is_our_placement", &GameState::ourPlacement)
        .def("is_their_kickoff", &GameState::theirKickoff)
        .def("is_their_penalty", &GameState::theirPenalty)
        .def("is_their_direct", &GameState::theirDirect)
        .def("is_their_indirect", &GameState::theirIndirect)
        .def("is_their_free_kick", &GameState::theirFreeKick)
        .def("is_their_placement", &GameState::theirPlacement)
        .def("is_setup_state", &GameState::inSetupState)
        .def("is_ready_state", &GameState::inReadyState)
        .def("can_kick", &GameState::canKick)
        .def("stay_away_from_ball", &GameState::stayAwayFromBall)
        .def("stay_on_side", &GameState::stayOnSide)
        .def("stay_behind_penalty_line", &GameState::stayBehindPenaltyLine)
        .def("is_our_restart", &GameState::isOurRestart)
        .def("get_ball_placement_point", &GameState::getBallPlacementPoint)
        .def("get_goalie_id", &GameState::getGoalieId);

    class_<Robot>("Robot", init<int, bool>())
        .def("shell_id", &Robot::shell)
        .def("is_ours", &Robot::self,
             "whether or not this robot is on our team")
        .add_property("pos", &Robot_pos,
                      "position vector of the robot in meters")
        .def("set_pos_for_testing", &Robot_set_pos_for_testing)
        .add_property("vel", &Robot_vel, "velocity vector of the robot in m/s")
        .add_property("angle", &Robot_angle, "angle of the robot in degrees")
        .add_property("angle_vel", &Robot_angle_vel,
                      "angular velocity in degrees per second")
        .add_property("visible", &Robot::visible)
        .def("__repr__", &Robot_repr)
        .def("__eq__", &Robot::operator==);

    class_<OurRobot, OurRobot*, bases<Robot>, boost::noncopyable>(
        "OurRobot", init<int, SystemState*>())
        .def("move_to", &OurRobot_move_to)
        .def("move_to_end_vel", &OurRobot_move_to_end_vel)
        .def("move_to_direct", &OurRobot_move_to_direct)
        .def("move_tuning", &OurRobot_move_tuning)
        .def("set_world_vel", &OurRobot::worldVelocity)
        .def("face", &OurRobot::face)
        .def("pivot", &OurRobot::pivot)
        .def("line_kick", &OurRobot::lineKick)
        .def("set_planning_priority", &OurRobot::setPlanningPriority)
        .def("set_max_angle_speed", OurRobot_set_max_angle_speed)
        .def("set_max_speed", OurRobot_set_max_speed)
        .def("set_avoid_ball_radius", &OurRobot_set_avoid_ball_radius)
        .def("disable_avoid_ball", &OurRobot::disableAvoidBall)
        .def("add_text", &OurRobot_add_text)
        .def("approach_opponent", &OurRobot_approach_opponent)
        .def("set_avoid_opponents", &OurRobot_set_avoid_opponents)
        .def("set_dribble_speed", &OurRobot::dribble)
        .def("has_ball", &OurRobot::hasBall)
        .def("has_ball_raw", &OurRobot::hasBallRaw)
        .def("last_kick_time", &OurRobot::lastKickTime)
        .def("just_kicked", &OurRobot::justKicked)
        .def("has_chipper", &OurRobot::chipper_available)
        .def("face_none", &OurRobot::faceNone)
        .def("kick", &OurRobot::kick)
        .def("kick_level", &OurRobot::kickLevel)
        .def("chip", &OurRobot::chip)
        .def("chip_level", &OurRobot::chipLevel)
        .def("unkick", &OurRobot::unkick,
             "clears any prevous kick command sent to the robot")
        .def("get_cmd_text", &OurRobot::getCmdText,
             "gets the string containing a list of commands sent to the robot, "
             "such as face(), move_to(), etc.")
        .def("ball_sense_works", &OurRobot::ballSenseWorks)
        .def("kicker_works", &OurRobot::kickerWorks)
        .def("add_local_obstacle", &OurRobot_add_local_obstacle)
        .def("initialize_tuner", &OurRobot_initialize_tuner)
        .def("start_pid_tuner", &OurRobot_start_pid_tuner)
        .def("run_pid_tuner", &OurRobot_run_pid_tuner)
        .def("end_pid_tuner", &OurRobot_end_pid_tuner)
        .def_readwrite("is_penalty_kicker", &OurRobot::isPenaltyKicker)
        .def_readwrite("is_ball_placer", &OurRobot::isBallPlacer);

    class_<OpponentRobot, OpponentRobot*, std::shared_ptr<OpponentRobot>,
           bases<Robot>>("OpponentRobot", init<int>());

    class_<Ball, std::shared_ptr<Ball>>("Ball", init<>())
        .def_readonly("pos", &Ball::pos)
        .def_readonly("vel", &Ball::vel)
        .def_readonly("valid", &Ball::valid);

    class_<std::vector<Robot*>>("vector_Robot")
        .def(vector_indexing_suite<std::vector<Robot*>>())
        .def("clear", &std::vector<Robot*>::clear);

    class_<std::vector<OurRobot*>>("vector_OurRobot")
        .def(vector_indexing_suite<std::vector<OurRobot*>>());

    class_<std::vector<OpponentRobot*>>("vector_OpponentRobot")
        .def(vector_indexing_suite<std::vector<OpponentRobot*>>());

    class_<SystemState, SystemState*>("SystemState")
        .def_readonly("our_robots", &SystemState::self)
        .def_readonly("their_robots", &SystemState::opp)
        .def_readonly("ball", &SystemState::ball)
        .def_readonly("game_state", &SystemState::gameState)

        .add_property("timestamp", &SystemState::timestamp)

        // debug drawing methods
        .def("draw_circle", &State_draw_circle)
        .def("draw_text", &State_draw_text)
        .def("draw_shape", &SystemState::drawShape)
        .def("draw_line", &State_draw_line)
        .def("draw_line", &State_draw_segment)
        .def("draw_segment", &State_draw_segment)
        .def("draw_polygon", &State_draw_polygon)
        .def("draw_arc", &State_draw_arc)
        .def("draw_raw_polygon", &State_draw_raw_polygon)
        .def("draw_arc", &State_draw_arc);

    class_<Field_Dimensions>("Field_Dimensions")
        .add_property("Length", &Field_Dimensions::Length)
        .add_property("Width", &Field_Dimensions::Width)
        .add_property("Border", &Field_Dimensions::Border)
        .add_property("LineWidth", &Field_Dimensions::LineWidth)
        .add_property("GoalWidth", &Field_Dimensions::GoalWidth)
        .add_property("GoalDepth", &Field_Dimensions::GoalDepth)
        .add_property("GoalHeight", &Field_Dimensions::GoalHeight)
        .add_property("PenaltyDist", &Field_Dimensions::PenaltyDist)
        .add_property("PenaltyDiam", &Field_Dimensions::PenaltyDiam)
        .add_property("ArcRadius", &Field_Dimensions::ArcRadius)
        .add_property("CenterRadius", &Field_Dimensions::CenterRadius)
        .add_property("CenterDiameter", &Field_Dimensions::CenterDiameter)
        .add_property("GoalFlat", &Field_Dimensions::GoalFlat)
        .add_property("FloorLength", &Field_Dimensions::FloorLength)
        .add_property("FloorWidth", &Field_Dimensions::FloorWidth)
        .add_property("CenterPoint", &Field_Dimensions::CenterPoint)
        .add_property("OurGoalZoneShape", &Field_Dimensions::OurGoalZoneShape)
        .add_property("TheirGoalZoneShape",
                      &Field_Dimensions::TheirGoalZoneShape)
        .add_property("OurGoalSegment", &Field_Dimensions::OurGoalSegment)
        .add_property("TheirGoalSegment", &Field_Dimensions::TheirGoalSegment)
        .add_property("OurHalf", &Field_Dimensions::OurHalf)
        .add_property("TheirHalf", &Field_Dimensions::TheirHalf)
        .add_property("FieldRect", &Field_Dimensions::FieldRect)
        .add_property("FieldBorders", &Field_Dimensions::FieldBorders)
        .def_readonly("SingleFieldDimensions",
                      &Field_Dimensions::Single_Field_Dimensions)
        .def_readonly("DoubleFieldDimensions",
                      &Field_Dimensions::Double_Field_Dimensions);

    class_<std::vector<Geometry2d::Line>>("vector_Line")
        .def(vector_indexing_suite<std::vector<Geometry2d::Line>>());

    class_<Window>("Window")
        .def_readwrite("a0", &Window::a0)
        .def_readwrite("a1", &Window::a1)
        .def_readwrite("t0", &Window::t0)
        .def_readwrite("t1", &Window::t1)
        .def_readwrite("segment", &Window::segment)
        .def_readwrite("shot_success", &Window::shot_success);

    class_<std::vector<Window>>("vector_Window")
        .def(vector_indexing_suite<std::vector<Window>>());

    class_<WindowEvaluator>("WindowEvaluator", init<SystemState*>())
        .def_readwrite("debug", &WindowEvaluator::debug)
        .def_readwrite("chip_enabled", &WindowEvaluator::chip_enabled)
        .def_readwrite("max_chip_range", &WindowEvaluator::max_chip_range)
        .def_readwrite("min_chip_range", &WindowEvaluator::min_chip_range)
        .def_readwrite("excluded_robots", &WindowEvaluator::excluded_robots)
        .def_readwrite("hypothetical_robot_locations",
                       &WindowEvaluator::hypothetical_robot_locations)
        .def("add_excluded_robot", &WinEval_add_excluded_robot)
        .def("eval_pt_to_pt", &WinEval_eval_pt_to_pt)
        .def("eval_pt_to_robot", &WinEval_eval_pt_to_robot)
        .def("eval_pt_to_opp_goal", &WinEval_eval_pt_to_opp_goal)
        .def("eval_pt_to_our_goal", &WinEval_eval_pt_to_our_goal)
        .def("eval_pt_to_seg", &WinEval_eval_pt_to_seg);

    class_<KickEvaluator>("KickEvaluator", init<SystemState*>())
        .def_readwrite("excluded_robots", &KickEvaluator::excluded_robots)
        .def_readwrite("hypothetical_robot_locations",
                       &KickEvaluator::hypothetical_robot_locations)
        .def("add_excluded_robot", &KickEval_add_excluded_robot)
        .def("eval_pt_to_pt", &KickEval_eval_pt_to_pt)
        .def("eval_pt_to_robot", &KickEval_eval_pt_to_robot)
        .def("eval_pt_to_opp_goal", &KickEval_eval_pt_to_opp_goal)
        .def("eval_pt_to_our_goal", &KickEval_eval_pt_to_our_goal)
        .def("eval_pt_to_seg", &KickEval_eval_pt_to_seg);

    class_<std::function<float(Geometry2d::Point)>,
           std::function<float(Geometry2d::Point)>*>("stdfunction", no_init)
        .def("__init__", make_constructor(&stdfunction_constructor));

    class_<NelderMead2DConfig>("NelderMead2DConfig", no_init)
        .def("__init__", make_constructor(&NelderMead2DConfig_constructor),
             "function is required")
        .def_readwrite("start", &NelderMead2DConfig::start)
        .def_readwrite("step", &NelderMead2DConfig::step)
        .def_readwrite("minDist", &NelderMead2DConfig::minDist)
        .def_readwrite("reflectionCoeff", &NelderMead2DConfig::reflectionCoeff)
        .def_readwrite("expansionCoeff", &NelderMead2DConfig::expansionCoeff)
        .def_readwrite("contractionCoeff",
                       &NelderMead2DConfig::contractionCoeff)
        .def_readwrite("shrinkCoeff", &NelderMead2DConfig::shrinkCoeff)
        .def_readwrite("maxIterations", &NelderMead2DConfig::maxIterations)
        .def_readwrite("maxValue", &NelderMead2DConfig::maxValue)
        .def_readwrite("maxThresh", &NelderMead2DConfig::maxThresh);

    class_<NelderMead2D>("NelderMead2D", no_init)
        .def("__init__", make_constructor(&NelderMead2D_constructor))
        .def("execute", &NelderMead2D::execute, "returns max value")
        .def("singleStep", &NelderMead2D::singleStep,
             "single run of optimization")
        .def("getValue", &NelderMead2D::getValue, "returns max value")
        .def("getPoint", &NelderMead2D::getPoint, "returns max point");

    class_<ConfigItem, ConfigItem*, boost::noncopyable>("ConfigItem", no_init)
        .def_readonly("name", &ConfigItem::name);

    class_<Configuration, std::shared_ptr<Configuration>, boost::noncopyable>(
        "Configuration")
        .def("FromRegisteredConfigurables",
             &Configuration::FromRegisteredConfigurables)
        .def("nameLookup", &Configuration::nameLookup,
             return_value_policy<reference_existing_object>())
        .staticmethod("FromRegisteredConfigurables");

    // Add wrappers for ConfigItem subclasses
    class_<ConfigBool, ConfigBool*, bases<ConfigItem>>("ConfigBool", no_init)
        .add_property("value", &ConfigBool::value, &ConfigBool::setValue)
        .def("__str__", &ConfigBool::toString);

    class_<ConfigDouble, ConfigDouble*, bases<ConfigItem>>("ConfigDouble",
                                                           no_init)
        .add_property("value", &ConfigDouble::value, &ConfigDouble::setValue)
        .def("__str__", &ConfigDouble::toString);

    class_<ConfigInt, ConfigInt*, bases<ConfigItem>>("ConfigInt", no_init)
        .add_property("value", &ConfigInt::value, &ConfigInt::setValue)
        .def("__str__", &ConfigInt::toString);

    class_<MotionConstraints>("MotionConstraints")
        .def_readonly("MaxRobotSpeed", &MotionConstraints::_max_speed)
        .def_readonly("MaxRobotAccel", &MotionConstraints::_max_acceleration);
}
