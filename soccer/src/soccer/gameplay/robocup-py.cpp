#include "robocup-py.hpp"

#include <functional>
#include <sstream>
#include <string>
#include <utility>

#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "world_state.hpp"

using namespace boost::python;

#include <exception>

#include <boost/python/exception_translator.hpp>
#include <boost/version.hpp>

#include <configuration.hpp>
#include <context.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/arc.hpp>
#include <rj_geometry/circle.hpp>
#include <rj_geometry/composite_shape.hpp>
#include <rj_geometry/line.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/polygon.hpp>
#include <rj_geometry/rect.hpp>
#include <rj_protos/LogFrame.pb.h>

#include "debug_drawer.hpp"
#include "kick_evaluator.hpp"
#include "control/motion_control.hpp"
#include "control/trapezoidal_motion.hpp"
#include "optimization/nelder_mead_2d.hpp"
#include "optimization/nelder_mead_2d_config.hpp"
#include "optimization/python_function_wrapper.hpp"
#include "planning/motion_constraints.hpp"
#include "referee/external_referee.hpp"
#include "robot.hpp"
#include "robot_config.hpp"
#include "system_state.hpp"
#include "window_evaluator.hpp"

#include <rc-fshare/pid.hpp>

/**
 * These functions make sure errors on the c++
 * side get passed up through python.
 */

struct NullArgumentException : public std::exception {
    std::string argument_name;
    std::string message;
    NullArgumentException() = default;
    NullArgumentException(std::string name)
        : argument_name{std::move(name)}, message{"'" + argument_name + "'' was 'None'."} {}
    [[nodiscard]] const char* what() const noexcept override { return message.c_str(); }
};

void translate_exception(NullArgumentException const& e) {
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

QColor color_from_tuple(const boost::python::tuple& rgb) {
    float r = extract<float>(rgb[0]);
    float g = extract<float>(rgb[1]);
    float b = extract<float>(rgb[2]);

    if (len(rgb) == 4) {
        float a = extract<float>(rgb[3]);
        return QColor(r, g, b, a);
    }
    return QColor(r, g, b);
}

std::string point_repr(rj_geometry::Point* self) { return self->to_string(); }

std::string robot_repr(Robot* self) { return self->to_string(); }

// Sets a robot's position - this should never be used in gameplay code, but
// is useful for testing.
void robot_set_pos_for_testing(Robot* self, rj_geometry::Point pos) {
    self->mutable_state().pose.position() = pos;
}

// Sets a robot's visibility - this should never be used in gameplay code, but
// is useful for testing.
void robot_set_vis_for_testing(Robot* self, bool vis) { self->mutable_state().visible = vis; }

// Sets a ball's position - this should never be used in gameplay code, but
// is useful for testing.
void ball_set_pos_for_testing(BallState* self, rj_geometry::Point pos) { self->position = pos; }

rj_geometry::Point robot_pos(Robot* self) { return self->pos(); }

rj_geometry::Point robot_vel(Robot* self) { return self->vel(); }

float robot_angle(Robot* self) { return self->angle(); }

float robot_angle_vel(Robot* self) { return self->angle_vel(); }

rj_geometry::Point ball_pos(BallState* self) { return self->position; }

rj_geometry::Point ball_vel(BallState* self) { return self->velocity; }

rj_geometry::Point ball_predict_pos(BallState* ball, double s) {
    return ball->predict_in(RJ::Seconds(s)).position;
}

double ball_estimate_seconds_to(BallState* ball, rj_geometry::Point p) {
    return ball->query_seconds_near(p).count();
}

double ball_predict_seconds_to_stop(BallState* ball) { return ball->query_stop_time().count(); }

double ball_estimate_seconds_to_dist(BallState* ball, double dist) {
    std::optional<RJ::Seconds> maybe_time = ball->query_seconds_to_dist(dist);
    if (!maybe_time.has_value()) {
        maybe_time = RJ::Seconds(std::numeric_limits<double>::infinity());
    }
    return maybe_time.value().count();
}

void our_robot_move_to_direct(OurRobot* self, rj_geometry::Point* to) {
    if (to == nullptr) {
        throw NullArgumentException("to");
    }
    self->move_direct(*to);
}

void our_robot_move_tuning(OurRobot* self, rj_geometry::Point* to) {
    if (to == nullptr) {
        throw NullArgumentException("to");
    }
    self->move_tuning(*to);
}

void our_robot_move_to_end_vel(OurRobot* self, rj_geometry::Point* end_pos, rj_geometry::Point* vf) {
    if (end_pos == nullptr || vf == nullptr) {
        throw NullArgumentException();
    }
    self->move(*end_pos, *vf);
}

void our_robot_move_to(OurRobot* self, rj_geometry::Point* to) {
    if (to == nullptr) {
        throw NullArgumentException("to");
    }
    self->move(*to);
}

void our_robot_settle(OurRobot* self) { self->settle(std::nullopt); }

void our_robot_settle_w_bounce(OurRobot* self, rj_geometry::Point* bounce_target) {
    if (bounce_target == nullptr) {
        throw NullArgumentException("bounce_target");
    }
    self->settle(*bounce_target);
}

void our_robot_add_local_obstacle(OurRobot* self, rj_geometry::Shape* obs) {
    if (obs == nullptr) {
        throw NullArgumentException("obs");
    }
    std::shared_ptr<rj_geometry::Shape> shared_obs(obs->clone());
    self->local_obstacles(shared_obs);
}

void our_robot_set_avoid_ball_radius([[maybe_unused]] OurRobot* self,
                                     [[maybe_unused]] float radius) {
    spdlog::warn("Configurable avoidance not implemented");
}

void our_robot_disable_avoid_ball([[maybe_unused]] OurRobot* self) {
    spdlog::warn("Configurable avoidance not implemented");
}

void our_robot_set_max_angle_speed(OurRobot* self, float max_angle_speed) {
    self->rotation_constraints().max_speed = max_angle_speed;
}

void our_robot_set_max_speed(OurRobot* self, float max_speed) {
    self->motion_constraints().max_speed = max_speed;
}

void our_robot_set_max_accel(OurRobot* self, float max_accel) {
    self->motion_constraints().max_acceleration = max_accel;
}

void our_robot_approach_opponent([[maybe_unused]] OurRobot* self,
                                 [[maybe_unused]] unsigned shell_id,
                                 [[maybe_unused]] bool enable_approach) {
    // TODO(Kyle): Use SPDLog
    std::cout << "Configurable avoidance not implemented" << std::endl;
}

void our_robot_add_text(OurRobot* self, const std::string& text, const boost::python::tuple& rgb,
                        const std::string& layer_prefix) {
    self->add_text(QString::fromStdString(text), color_from_tuple(rgb),
                   QString::fromStdString(layer_prefix));
}

void our_robot_set_avoid_opponents([[maybe_unused]] OurRobot* self, [[maybe_unused]] bool value) {
    // TODO(Kyle): Use SPDLog
    std::cout << "Configurable avoidance not implemented" << std::endl;
}

// Tuner code disabled pending refactor since it was removed from
// the firmware shared repo
void our_robot_initialize_tuner(OurRobot* self, char controller) {
    // self->motion_control()->get_pid(controller)->initialize_tuner();
}

void our_robot_start_pid_tuner(OurRobot* self, char controller) {
    // self->motion_control()->get_pid(controller)->start_tuner_cycle();
    // self->config->translation.p->set_value(
    // self->motion_control()->get_pid(controller)->kp);
    // self->config->translation.i->set_value(
    // self->motion_control()->get_pid(controller)->ki);
    // self->config->translation.d->set_value(
    // self->motion_control()->get_pid(controller)->kd);
}

void our_robot_run_pid_tuner(OurRobot* self, char controller) {
    // self->motion_control()->get_pid(controller)->run_tuner();
}

bool our_robot_end_pid_tuner(OurRobot* /*self*/, char /*controller*/) {
    // return self->motion_control()->get_pid(controller)->end_tuner_cycle();
    return false;
}

bool rect_contains_rect(rj_geometry::Rect* self, rj_geometry::Rect* other) {
    if (other == nullptr) {
        throw NullArgumentException("other");
    }
    return self->contains_rect(*other);
}

bool rect_contains_point(rj_geometry::Rect* self, rj_geometry::Point* pt) {
    if (pt == nullptr) {
        throw NullArgumentException("pt");
    }
    return self->contains_point(*pt);
}

void point_rotate(rj_geometry::Point* self, rj_geometry::Point* origin, float angle) {
    if (origin == nullptr) {
        throw NullArgumentException("origin");
    }
    self->rotate(*origin, angle);
}

void point_rotate_origin(rj_geometry::Point* self, float angle) { self->rotate(angle); }

void composite_shape_add_shape(rj_geometry::CompositeShape* self, rj_geometry::Shape* shape) {
    if (shape == nullptr) {
        throw NullArgumentException("shape");
    }
    self->add(std::shared_ptr<rj_geometry::Shape>(shape->clone()));
}

void polygon_add_vertex(rj_geometry::Polygon* self, rj_geometry::Point* pt) {
    if (pt == nullptr) {
        throw NullArgumentException("pt");
    }
    self->add_vertex(*pt);
}

rj_geometry::Point* line_get_pt(rj_geometry::Line* self, int index) { return &(self->pt[index]); }

rj_geometry::Point* segment_get_pt(rj_geometry::Segment* self, int index) {
    return &(self->pt[index]);
}

rj_geometry::Point* rect_get_pt(rj_geometry::Rect* self, int index) { return &(self->pt[index]); }

boost::python::object segment_segment_intersection(rj_geometry::Segment* self,
                                                   rj_geometry::Segment* other) {
    if (other == nullptr) {
        throw NullArgumentException("other");
    }
    rj_geometry::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
}

bool rect_rect_intersection(rj_geometry::Rect* self, rj_geometry::Rect* other) {
    if (other == nullptr) {
        throw NullArgumentException{"other"};
    }
    return self->intersects(*other);
}

boost::python::object rect_segment_intersection(rj_geometry::Rect* self,
                                                rj_geometry::Segment* segment) {
    if (segment == nullptr) {
        throw NullArgumentException{"segment"};
    }
    boost::python::list lst;
    std::tuple<bool, std::vector<rj_geometry::Point>> result = self->intersects(*segment);
    bool does_intersect = std::get<0>(result);
    if (!does_intersect) {
        return boost::python::object();
    }

    std::vector<rj_geometry::Point> intersection_points = std::get<1>(result);
    std::vector<rj_geometry::Point>::iterator it;
    for (it = intersection_points.begin(); it != intersection_points.end(); it++) {
        lst.append(*it);
    }
    return lst;
}

boost::python::object rect_corners(rj_geometry::Rect* self) {
    boost::python::list lst;
    std::vector<rj_geometry::Point> corners = self->corners();
    std::vector<rj_geometry::Point>::iterator it;
    for (it = corners.begin(); it != corners.end(); it++) {
        lst.append(*it);
    }
    return lst;
}

boost::python::object segment_line_intersection(rj_geometry::Segment* self, rj_geometry::Line* line) {
    if (line == nullptr) {
        throw NullArgumentException{"line"};
    }
    rj_geometry::Point pt;
    if (self->intersects(*line, &pt)) {
        boost::python::object obj{pt};
        return obj;
    } else {
        return boost::python::object{};
    }
}

boost::python::object segment_nearest_point_to_point(rj_geometry::Segment* self,
                                                     rj_geometry::Point* point) {
    if (point == nullptr) {
        throw NullArgumentException{"point"};
    }
    return boost::python::object{self->nearest_point(*point)};
}

boost::python::object segment_nearest_point_to_line(rj_geometry::Segment* self,
                                                    rj_geometry::Line* line) {
    if (line == nullptr) {
        throw NullArgumentException{"line"};
    }
    return boost::python::object{self->nearest_point(*line)};
}

// returns None or a rj_geometry::Point
boost::python::object line_line_intersection(rj_geometry::Line* self, rj_geometry::Line* other) {
    if (other == nullptr) {
        throw NullArgumentException("other");
    }
    rj_geometry::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
};

boost::python::object line_segment_intersection(rj_geometry::Line* self,
                                                rj_geometry::Segment* other) {
    if (other == nullptr) {
        throw NullArgumentException("other");
    }
    rj_geometry::Point pt;
    if (self->intersects(*other, &pt)) {
        boost::python::object obj(pt);
        return obj;
    } else {
        // return None
        return boost::python::object();
    }
};

boost::python::tuple line_intersects_circle(rj_geometry::Line* self, rj_geometry::Circle* circle) {
    if (circle == nullptr) {
        throw NullArgumentException("circle");
    }
    rj_geometry::Point a;
    rj_geometry::Point b;
    boost::python::list lst;

    bool intersects = self->intersects(*circle, &a, &b);
    lst.append(intersects);
    lst.append(a);
    lst.append(b);

    return boost::python::tuple(lst);
}

void debug_drawer_draw_circle(DebugDrawer* self, const rj_geometry::Point* center, float radius,
                              const boost::python::tuple& rgb, const std::string& layer) {
    if (center == nullptr) {
        throw NullArgumentException("center");
    }
    self->draw_circle(*center, radius, color_from_tuple(rgb), QString::fromStdString(layer));
}

void debug_drawer_draw_arc(DebugDrawer* self, const rj_geometry::Arc* arc,
                           const boost::python::tuple& rgb, const std::string& layer) {
    if (arc == nullptr) {
        throw NullArgumentException{"arc"};
    }
    self->draw_arc(*arc, color_from_tuple(rgb), QString::fromStdString(layer));
}

// TODO(ashaw596) Fix this lie of a function
void debug_drawer_draw_line(DebugDrawer* self, const rj_geometry::Line* line,
                            const boost::python::tuple& rgb, const std::string& layer) {
    if (line == nullptr) {
        throw NullArgumentException("line");
    }
    self->draw_line(rj_geometry::Segment(*line), color_from_tuple(rgb),
                    QString::fromStdString(layer));
}

void debug_drawer_draw_segment(DebugDrawer* self, const rj_geometry::Segment* segment,
                               const boost::python::tuple& rgb, const std::string& layer) {
    if (segment == nullptr) {
        throw NullArgumentException("segment");
    }
    self->draw_segment(*segment, color_from_tuple(rgb), QString::fromStdString(layer));
}

void debug_drawer_draw_segment_from_points(DebugDrawer* self, const rj_geometry::Point* p0,
                                           const rj_geometry::Point* p1,
                                           const boost::python::tuple& rgb,
                                           const std::string& layer) {
    if (p0 == nullptr) {
        throw NullArgumentException{"p0"};
    }
    if (p1 == nullptr) {
        throw NullArgumentException{"p1"};
    }
    self->draw_line(*p0, *p1, color_from_tuple(rgb), QString::fromStdString(layer));
}

void debug_drawer_draw_text(DebugDrawer* self, const std::string& text, rj_geometry::Point* pos,
                            const boost::python::tuple& rgb, const std::string& layer) {
    if (pos == nullptr) {
        throw NullArgumentException("position");
    }
    self->draw_text(QString::fromStdString(text), *pos, color_from_tuple(rgb),
                    QString::fromStdString(layer));
}

void debug_drawer_draw_polygon(DebugDrawer* self, const boost::python::list& points,
                               const boost::python::tuple& rgb, const std::string& layer) {
    std::vector<rj_geometry::Point> pt_vec;
    for (int i = 0; i < len(points); i++) {
        pt_vec.push_back(boost::python::extract<rj_geometry::Point>(points[i]));
    }

    self->draw_polygon(pt_vec, color_from_tuple(rgb), QString::fromStdString(layer));
}

void debug_drawer_draw_raw_polygon(DebugDrawer* self, const rj_geometry::Polygon& points,
                                   const boost::python::tuple& rgb, const std::string& layer) {
    self->draw_polygon(points, color_from_tuple(rgb), QString::fromStdString(layer));
}

boost::python::list circle_intersects_line(rj_geometry::Circle* self, const rj_geometry::Line* line) {
    if (line == nullptr) {
        throw NullArgumentException("line");
    }
    boost::python::list lst;

    rj_geometry::Point intersection_points[2];
    int num_intersects = self->intersects(*line, intersection_points);
    for (int i = 0; i < num_intersects; i++) {
        lst.append(intersection_points[i]);
    }

    return lst;
}

boost::python::list arc_intersects_line(rj_geometry::Arc* self, const rj_geometry::Line* line) {
    if (line == nullptr) {
        throw NullArgumentException{"line"};
    }
    boost::python::list lst;

    auto intersections = self->intersects(*line);

    for (auto& intersection : intersections) {
        lst.append(intersection);
    }

    return lst;
}

boost::python::list arc_intersects_segment(rj_geometry::Arc* self,
                                           const rj_geometry::Segment* segment) {
    if (segment == nullptr) {
        throw NullArgumentException{"segment"};
    }
    boost::python::list lst;

    auto intersections = self->intersects(*segment);

    for (auto& intersection : intersections) {
        lst.append(intersection);
    }

    return lst;
}

boost::python::tuple win_eval_eval_pt_to_seg(WindowEvaluator* self, const rj_geometry::Point* origin,
                                             const rj_geometry::Segment* target) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto window_results = self->eval_pt_to_seg(*origin, *target);

    lst.append(window_results.first);
    if (window_results.second.has_value())
        lst.append(window_results.second.value());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple win_eval_eval_pt_to_robot(WindowEvaluator* self,
                                               const rj_geometry::Point* origin,
                                               const rj_geometry::Point* target) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto window_results = self->eval_pt_to_robot(*origin, *target);

    lst.append(window_results.first);
    if (window_results.second.has_value())
        lst.append(window_results.second.value());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple win_eval_eval_pt_to_pt(WindowEvaluator* self, const rj_geometry::Point* origin,
                                            const rj_geometry::Point* target, float target_width) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto window_results = self->eval_pt_to_pt(*origin, *target, target_width);

    lst.append(window_results.first);
    if (window_results.second.has_value())
        lst.append(window_results.second.value());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple win_eval_eval_pt_to_opp_goal(WindowEvaluator* self,
                                                  const rj_geometry::Point* origin) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    boost::python::list lst;

    auto window_results = self->eval_pt_to_opp_goal(*origin);

    lst.append(window_results.first);
    if (window_results.second.has_value())
        lst.append(window_results.second.value());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

boost::python::tuple win_eval_eval_pt_to_our_goal(WindowEvaluator* self,
                                                  const rj_geometry::Point* origin) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    boost::python::list lst;

    auto window_results = self->eval_pt_to_our_goal(*origin);

    lst.append(window_results.first);
    if (window_results.second.has_value())
        lst.append(window_results.second.value());
    else
        lst.append(boost::python::api::object());

    return boost::python::tuple{lst};
}

void win_eval_add_excluded_robot(WindowEvaluator* self, Robot* robot) {
    self->excluded_robots.push_back(robot);
}

boost::python::tuple kick_eval_eval_pt_to_seg(KickEvaluator* self, const rj_geometry::Point* origin,
                                              const rj_geometry::Segment* target) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_seg(*origin, *target);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple kick_eval_eval_pt_to_robot(KickEvaluator* self,
                                                const rj_geometry::Point* origin,
                                                const rj_geometry::Point* target) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_robot(*origin, *target);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple kick_eval_eval_pt_to_pt(KickEvaluator* self, const rj_geometry::Point* origin,
                                             const rj_geometry::Point* target, float target_width) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    if (target == nullptr) {
        throw NullArgumentException{"target"};
    }
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_pt(*origin, *target, target_width);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple kick_eval_eval_pt_to_opp_goal(KickEvaluator* self,
                                                   const rj_geometry::Point* origin) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_opp_goal(*origin);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

boost::python::tuple kick_eval_eval_pt_to_our_goal(KickEvaluator* self,
                                                   const rj_geometry::Point* origin) {
    if (origin == nullptr) {
        throw NullArgumentException{"origin"};
    }
    boost::python::list lst;

    auto kick_results = self->eval_pt_to_our_goal(*origin);

    lst.append(kick_results.first);
    lst.append(kick_results.second);

    return boost::python::tuple{lst};
}

void kick_eval_add_excluded_robot(KickEvaluator* self, Robot* robot) {
    self->excluded_robots.push_back(robot);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Point_overloads, normalized, 0, 1)

boost::shared_ptr<PythonFunctionWrapper> python_function_wrapper_constructor(PyObject* pf) {
    return boost::shared_ptr<PythonFunctionWrapper>(new PythonFunctionWrapper(pf));
}

float point_get_x(const rj_geometry::Point* self) { return self->x(); }
float point_get_y(const rj_geometry::Point* self) { return self->y(); }
void point_set_x(rj_geometry::Point* self, float x) { self->x() = x; }
void point_set_y(rj_geometry::Point* self, float y) { self->y() = y; }

boost::shared_ptr<NelderMead2DConfig> nelder_mead_2d_config_constructor(
    PythonFunctionWrapper* function_wrapper, rj_geometry::Point start = rj_geometry::Point(0, 0),
    rj_geometry::Point step = rj_geometry::Point(1, 1),
    rj_geometry::Point min_dist = rj_geometry::Point(0.001, 0.001), float reflection_coeff = 1,
    float expansion_coeff = 2, float contraction_coeff = 0.5, float shrink_coeff = 0.5,
    int max_iterations = 100, float max_value = 0, float max_thresh = 0) {
    return boost::shared_ptr<NelderMead2DConfig>(new NelderMead2DConfig(
        function_wrapper->f, start, step, min_dist, reflection_coeff, expansion_coeff,
        contraction_coeff, shrink_coeff, max_iterations, max_value, max_thresh));
}

boost::shared_ptr<NelderMead2D> nelder_mead_2d_constructor(NelderMead2DConfig* config) {
    return boost::shared_ptr<NelderMead2D>(new NelderMead2D(*config));
}

/**
 * The code in this block wraps up c++ classes and makes them
 * accessible to python in the 'robocup' module.
 */
BOOST_PYTHON_MODULE(robocup) {
    boost::python::register_exception_translator<NullArgumentException>(&translate_exception);

    def("fix_angle_radians", &fix_angle_radians<double>);
    def("get_trapezoidal_time", &Trapezoidal::get_time);

    class_<rj_geometry::Point, rj_geometry::Point*>("Point", init<float, float>())
        .def(init<const rj_geometry::Point&>())
        .add_property("x", &point_get_x, &point_set_x)
        .add_property("y", &point_get_y, &point_set_y)
        .def(self - self)
        .def(self + self)
        .def(self * self)
        .def("mag", &rj_geometry::Point::mag)
        .def("magsq", &rj_geometry::Point::magsq)
        .def("__repr__", &point_repr)
        .def("normalized", &rj_geometry::Point::normalized, Point_overloads())
        .def("rotate", &point_rotate)
        .def("rotate_origin", &point_rotate_origin)
        .def(self * float())
        .def(self / float())
        .def("perp_ccw", &rj_geometry::Point::perp_ccw)
        .def("perp_cw", &rj_geometry::Point::perp_cw)
        .def("angle", &rj_geometry::Point::angle)
        .def("dot", &rj_geometry::Point::dot)
        .def("near_point", &rj_geometry::Point::near_point)
        .def("dist_to", &rj_geometry::Point::dist_to)
        .def("direction", &rj_geometry::Point::direction)
        .def("angle_between", &rj_geometry::Point::angle_between)
        .def("nearly_equals", &rj_geometry::Point::nearly_equals)
        .staticmethod("direction");

    class_<std::vector<rj_geometry::Point>>("vector_Point")
        .def(vector_indexing_suite<std::vector<rj_geometry::Point>>());

    class_<rj_geometry::Line, rj_geometry::Line*>("Line",
                                                init<rj_geometry::Point, rj_geometry::Point>())
        .def("delta", &rj_geometry::Line::delta)
        .def("line_intersection", &line_line_intersection)
        .def("segment_intersection", &line_segment_intersection)
        .def("dist_to", &rj_geometry::Line::dist_to)
        .def("intersects_circle", &line_intersects_circle)
        .def("get_pt", &line_get_pt, return_value_policy<reference_existing_object>())
        .def("nearest_point", &rj_geometry::Line::nearest_point);

    class_<rj_geometry::Segment, rj_geometry::Segment*>("Segment",
                                                      init<rj_geometry::Point, rj_geometry::Point>())
        .def("center", &rj_geometry::Segment::center)
        .def("length", &rj_geometry::Segment::length)
        .def("dist_to", &rj_geometry::Segment::dist_to)
        .def("get_pt", &segment_get_pt, return_value_policy<reference_existing_object>())
        .def("nearest_point", &segment_nearest_point_to_point)
        .def("segment_intersection", &segment_segment_intersection)
        .def("line_intersection", &segment_line_intersection)
        .def("near_point", &rj_geometry::Segment::near_point)
        .def("nearest_point_to_line", &segment_nearest_point_to_line)
        .def("__str__", &rj_geometry::Segment::to_string);

    class_<rj_geometry::Shape, boost::noncopyable>("Shape");

    class_<rj_geometry::Rect, bases<rj_geometry::Shape>>("Rect",
                                                       init<rj_geometry::Point, rj_geometry::Point>())
        .def("contains_rect", &rect_contains_rect)
        .def("min_x", &rj_geometry::Rect::minx)
        .def("min_y", &rj_geometry::Rect::miny)
        .def("max_x", &rj_geometry::Rect::maxx)
        .def("max_y", &rj_geometry::Rect::maxy)
        .def("corners", &rect_corners)
        .def("pad", &rj_geometry::Rect::pad)
        .def("near_point", &rj_geometry::Rect::near_point)
        .def("rect_intersection", &rect_rect_intersection)
        .def("segment_intersection", &rect_segment_intersection)
        .def("contains_point", &rj_geometry::Rect::contains_point)
        .def("get_pt", &rect_get_pt, return_value_policy<reference_existing_object>());

    class_<rj_geometry::Circle, bases<rj_geometry::Shape>>("Circle", init<rj_geometry::Point, float>())
        .def("intersects_line", &circle_intersects_line)
        .def("nearest_point", &rj_geometry::Circle::nearest_point)
        .def("contains_point", &rj_geometry::Circle::contains_point)
        .def_readonly("center", &rj_geometry::Circle::center);

    class_<rj_geometry::Arc>("Arc", init<rj_geometry::Point, float, float, float>())
        .def("intersects_line", &arc_intersects_line)
        .def("intersects_segment", &arc_intersects_segment)
        .def("center", &rj_geometry::Arc::center)
        .def("radius", &rj_geometry::Arc::radius)
        .def("start", &rj_geometry::Arc::start)
        .def("end", &rj_geometry::Arc::end);

    class_<rj_geometry::CompositeShape, bases<rj_geometry::Shape>>("CompositeShape", init<>())
        .def("clear", &rj_geometry::CompositeShape::clear)
        .def("is_empty", &rj_geometry::CompositeShape::empty)
        .def("size", &rj_geometry::CompositeShape::size)
        .def("add_shape", &composite_shape_add_shape)
        .def("contains_point", &rj_geometry::CompositeShape::contains_point);

    class_<rj_geometry::Polygon, bases<rj_geometry::Shape>>("Polygon", init<>())
        .def("add_vertex", &polygon_add_vertex)
        .def("contains_point", &rj_geometry::Polygon::contains_point);

    class_<GameState>("GameState")
        .def("is_halted", &GameState::halt)
        .def("is_stopped", &GameState::stopped)
        .def("is_playing", &GameState::playing)
        .def("is_kickoff", &GameState::kickoff)
        .def("is_penalty", &GameState::penalty)
        .def("is_placement", &GameState::placement)
        .def("is_direct", &GameState::direct)
        .def("is_indirect", &GameState::indirect)
        .def("is_our_kickoff", &GameState::our_kickoff)
        .def("is_our_penalty", &GameState::our_penalty)
        .def("is_our_direct", &GameState::our_direct)
        .def("is_our_indirect", &GameState::our_indirect)
        .def("is_our_free_kick", &GameState::our_free_kick)
        .def("is_our_placement", &GameState::our_placement)
        .def("is_their_kickoff", &GameState::their_kickoff)
        .def("is_their_penalty", &GameState::their_penalty)
        .def("is_their_direct", &GameState::their_direct)
        .def("is_their_indirect", &GameState::their_indirect)
        .def("is_their_free_kick", &GameState::their_free_kick)
        .def("is_their_placement", &GameState::their_placement)
        .def("is_setup_state", &GameState::in_setup_state)
        .def("is_ready_state", &GameState::in_ready_state)
        .def("can_kick", &GameState::can_kick)
        .def("stay_away_from_ball", &GameState::stay_away_from_ball)
        .def("stay_on_side", &GameState::stay_on_side)
        .def("stay_behind_penalty_line", &GameState::stay_behind_penalty_line)
        .def("is_our_restart", &GameState::is_our_restart)
        .def("get_ball_placement_point", &GameState::get_ball_placement_point)
        .def("is_first_half", &GameState::is_first_half)
        .def("is_second_half", &GameState::is_second_half)
        .def("is_halftime", &GameState::is_halftime)
        .def("is_overtime1", &GameState::is_overtime1)
        .def("is_overtime2", &GameState::is_overtime2)
        .def("is_penalty_shootout", &GameState::is_penalty_shootout);

    class_<TeamInfo>("TeamInfo")
        .add_property("score", &TeamInfo::score)
        .add_property("goalie", &TeamInfo::goalie);

    class_<Robot>("Robot", init<Context*, int, bool>())
        .def("shell_id", &Robot::shell)
        .def("is_ours", &Robot::self, "whether or not this robot is on our team")
        .add_property("pos", &robot_pos, "position vector of the robot in meters")
        .def("set_pos_for_testing", &robot_set_pos_for_testing)
        .def("set_vis_for_testing", &robot_set_vis_for_testing)
        .add_property("vel", &robot_vel, "velocity vector of the robot in m/s")
        .add_property("angle", &robot_angle, "angle of the robot in degrees")
        .add_property("angle_vel", &robot_angle_vel, "angular velocity in degrees per second")
        .add_property("visible", &Robot::visible)
        .def("__repr__", &robot_repr)
        .def("__eq__", &Robot::operator==);

    class_<OurRobot, OurRobot*, bases<Robot>, boost::noncopyable>("OurRobot", init<Context*, int>())
        .def("move_to", &our_robot_move_to)
        .def("move_to_end_vel", &our_robot_move_to_end_vel)
        .def("move_to_direct", &our_robot_move_to_direct)
        .def("move_tuning", &our_robot_move_tuning)
        .def("settle", &our_robot_settle)
        .def("settle_w_bounce", &our_robot_settle_w_bounce)
        .def("collect", &OurRobot::collect)
        .def("set_world_vel", &OurRobot::world_velocity)
        .def("face", &OurRobot::face)
        .def("pivot", &OurRobot::pivot)
        .def("line_kick", &OurRobot::line_kick)
        .def("intercept", &OurRobot::intercept)
        .def("set_planning_priority", &OurRobot::set_planning_priority)
        .def("set_max_angle_speed", our_robot_set_max_angle_speed)
        .def("set_max_speed", our_robot_set_max_speed)
        .def("set_max_accel", our_robot_set_max_accel)
        .def("set_avoid_ball_radius", &our_robot_set_avoid_ball_radius)
        .def("disable_avoid_ball", &our_robot_disable_avoid_ball)
        .def("add_text", &our_robot_add_text)
        .def("approach_opponent", &our_robot_approach_opponent)
        .def("set_avoid_opponents", &our_robot_set_avoid_opponents)
        .def("set_dribble_speed", &OurRobot::dribble)
        .def("has_ball", &OurRobot::has_ball)
        .def("has_ball_raw", &OurRobot::has_ball_raw)
        .def("last_kick_time", &OurRobot::last_kick_time)
        .def("just_kicked", &OurRobot::just_kicked)
        .def("has_chipper", &OurRobot::chipper_available)
        .def("kick", &OurRobot::kick)
        .def("kick_level", &OurRobot::kick_level)
        .def("chip", &OurRobot::chip)
        .def("chip_level", &OurRobot::chip_level)
        .def("unkick", &OurRobot::unkick, "clears any prevous kick command sent to the robot")
        .def("kick_immediately", &OurRobot::kick_immediately)
        .def("get_cmd_text", &OurRobot::get_cmd_text,
             "gets the string containing a list of commands sent to the robot, "
             "such as face(), move_to(), etc.")
        .def("ball_sense_works", &OurRobot::ball_sense_works)
        .def("kicker_works", &OurRobot::kicker_works)
        .def("add_local_obstacle", &our_robot_add_local_obstacle)
        .def("initialize_tuner", &our_robot_initialize_tuner)
        .def("start_pid_tuner", &our_robot_start_pid_tuner)
        .def("run_pid_tuner", &our_robot_run_pid_tuner)
        .def("end_pid_tuner", &our_robot_end_pid_tuner)
        .def_readwrite("is_penalty_kicker", &OurRobot::is_penalty_kicker)
        .def_readwrite("is_ball_placer", &OurRobot::is_ball_placer);

    class_<OpponentRobot, OpponentRobot*, std::shared_ptr<OpponentRobot>, bases<Robot>>(
        "OpponentRobot", init<Context*, int>());

    class_<BallState, BallState*>("Ball", init<>())
        .def("set_pos_for_testing", &ball_set_pos_for_testing)
        .add_property("pos", &ball_pos)
        .add_property("vel", &ball_vel)
        .def_readonly("valid", &BallState::visible)
        .def("predict_pos", &ball_predict_pos)
        .def("estimate_seconds_to", &ball_estimate_seconds_to)
        .def("predict_seconds_to_stop", &ball_predict_seconds_to_stop)
        .def("estimate_seconds_to_dist", &ball_estimate_seconds_to_dist);

    class_<std::vector<Robot*>>("vector_Robot")
        .def(vector_indexing_suite<std::vector<Robot*>>())
        .def("clear", &std::vector<Robot*>::clear);

    class_<std::vector<OurRobot*>>("vector_OurRobot")
        .def(vector_indexing_suite<std::vector<OurRobot*>>());

    class_<std::vector<OpponentRobot*>>("vector_OpponentRobot")
        .def(vector_indexing_suite<std::vector<OpponentRobot*>>());

    class_<SystemState, SystemState*>("SystemState", init<Context*>())
        .def_readonly("our_robots", &SystemState::self)
        .def_readonly("their_robots", &SystemState::opp)
        .def_readonly("ball", &SystemState::ball)
        .add_property("timestamp", &SystemState::timestamp);

    // debug drawing methods
    class_<DebugDrawer, DebugDrawer*>("DebugDrawer", init<Context*>())
        .def("draw_circle", &debug_drawer_draw_circle)
        .def("draw_text", &debug_drawer_draw_text)
        .def("draw_shape", &DebugDrawer::draw_shape)
        .def("draw_line", &debug_drawer_draw_line)
        .def("draw_line", &debug_drawer_draw_segment)
        .def("draw_segment", &debug_drawer_draw_segment)
        .def("draw_polygon", &debug_drawer_draw_polygon)
        .def("draw_arc", &debug_drawer_draw_arc)
        .def("draw_raw_polygon", &debug_drawer_draw_raw_polygon)
        .def("draw_arc", &debug_drawer_draw_arc);

    class_<Context, Context*, boost::noncopyable>("Context")
        .def_readonly("state", &Context::state)
        .def_readonly("debug_drawer", &Context::debug_drawer)
        .def_readonly("our_info", &Context::our_info)
        .def_readonly("their_info", &Context::their_info)
        .def_readonly("game_state", &Context::game_state);

    class_<FieldDimensions>("FieldDimensions")
        .def("OurGoalZoneShapePadded", &FieldDimensions::our_goal_zone_shape_padded)
        .add_property("Length", &FieldDimensions::length)
        .add_property("Width", &FieldDimensions::width)
        .add_property("Border", &FieldDimensions::border)
        .add_property("LineWidth", &FieldDimensions::line_width)
        .add_property("GoalWidth", &FieldDimensions::goal_width)
        .add_property("GoalDepth", &FieldDimensions::goal_depth)
        .add_property("GoalHeight", &FieldDimensions::goal_height)
        .add_property("PenaltyShortDist", &FieldDimensions::penalty_short_dist)
        .add_property("PenaltyLongDist", &FieldDimensions::penalty_long_dist)
        .add_property("CenterRadius", &FieldDimensions::center_radius)
        .add_property("CenterDiameter", &FieldDimensions::center_diameter)
        .add_property("GoalFlat", &FieldDimensions::goal_flat)
        .add_property("FloorLength", &FieldDimensions::floor_length)
        .add_property("FloorWidth", &FieldDimensions::floor_width)
        .add_property("CenterPoint", &FieldDimensions::center_point)
        .add_property("OurGoalZoneShape", &FieldDimensions::our_goal_zone_shape)
        .add_property("TheirGoalZoneShape", &FieldDimensions::their_goal_zone_shape)
        .add_property("OurGoalSegment", &FieldDimensions::our_goal_segment)
        .add_property("TheirGoalSegment", &FieldDimensions::their_goal_segment)
        .add_property("OurHalf", &FieldDimensions::our_half)
        .add_property("TheirHalf", &FieldDimensions::their_half)
        .add_property("FieldRect", &FieldDimensions::field_rect)
        .add_property("FieldBorders", &FieldDimensions::field_borders)
        .def_readonly("SingleFieldDimensions", &FieldDimensions::kSingleFieldDimensions)
        .def_readonly("DoubleFieldDimensions", &FieldDimensions::kDoubleFieldDimensions)
        .def_readonly("CurrentDimensions", &FieldDimensions::current_dimensions);

    class_<std::vector<rj_geometry::Line>>("vector_Line")
        .def(vector_indexing_suite<std::vector<rj_geometry::Line>>());

    class_<Window>("Window")
        .def_readwrite("a0", &Window::a0)
        .def_readwrite("a1", &Window::a1)
        .def_readwrite("t0", &Window::t0)
        .def_readwrite("t1", &Window::t1)
        .def_readwrite("segment", &Window::segment)
        .def_readwrite("shot_success", &Window::shot_success);

    class_<std::vector<Window>>("vector_Window").def(vector_indexing_suite<std::vector<Window>>());

    class_<WindowEvaluator>("WindowEvaluator", init<Context*>())
        .def_readwrite("debug", &WindowEvaluator::debug)
        .def_readwrite("chip_enabled", &WindowEvaluator::chip_enabled)
        .def_readwrite("max_chip_range", &WindowEvaluator::max_chip_range)
        .def_readwrite("min_chip_range", &WindowEvaluator::min_chip_range)
        .def_readwrite("excluded_robots", &WindowEvaluator::excluded_robots)
        .def_readwrite("hypothetical_robot_locations",
                       &WindowEvaluator::hypothetical_robot_locations)
        .def("add_excluded_robot", &win_eval_add_excluded_robot)
        .def("eval_pt_to_pt", &win_eval_eval_pt_to_pt)
        .def("eval_pt_to_robot", &win_eval_eval_pt_to_robot)
        .def("eval_pt_to_opp_goal", &win_eval_eval_pt_to_opp_goal)
        .def("eval_pt_to_our_goal", &win_eval_eval_pt_to_our_goal)
        .def("eval_pt_to_seg", &win_eval_eval_pt_to_seg);

    class_<KickEvaluator>("KickEvaluator", init<SystemState*>())
        .def_readwrite("excluded_robots", &KickEvaluator::excluded_robots)
        .def_readwrite("hypothetical_robot_locations", &KickEvaluator::hypothetical_robot_locations)
        .def("add_excluded_robot", &kick_eval_add_excluded_robot)
        .def("eval_pt_to_pt", &kick_eval_eval_pt_to_pt)
        .def("eval_pt_to_robot", &kick_eval_eval_pt_to_robot)
        .def("eval_pt_to_opp_goal", &kick_eval_eval_pt_to_opp_goal)
        .def("eval_pt_to_our_goal", &kick_eval_eval_pt_to_our_goal)
        .def("eval_pt_to_seg", &kick_eval_eval_pt_to_seg);

    class_<PythonFunctionWrapper>("PythonFunctionWrapper", no_init)
        .def("__init__", make_constructor(&python_function_wrapper_constructor));

    class_<NelderMead2DConfig>("NelderMead2DConfig", no_init)
        .def("__init__", make_constructor(&nelder_mead_2d_config_constructor),
             "function is required")
        .def_readwrite("start", &NelderMead2DConfig::start)
        .def_readwrite("step", &NelderMead2DConfig::step)
        .def_readwrite("minDist", &NelderMead2DConfig::min_dist)
        .def_readwrite("reflectionCoeff", &NelderMead2DConfig::reflection_coeff)
        .def_readwrite("expansionCoeff", &NelderMead2DConfig::expansion_coeff)
        .def_readwrite("contractionCoeff", &NelderMead2DConfig::contraction_coeff)
        .def_readwrite("shrinkCoeff", &NelderMead2DConfig::shrink_coeff)
        .def_readwrite("maxIterations", &NelderMead2DConfig::max_iterations)
        .def_readwrite("maxValue", &NelderMead2DConfig::max_value)
        .def_readwrite("maxThresh", &NelderMead2DConfig::max_thresh);

    class_<NelderMead2D>("NelderMead2D", no_init)
        .def("__init__", make_constructor(&nelder_mead_2d_constructor))
        .def("execute", &NelderMead2D::execute, "returns max value")
        .def("singleStep", &NelderMead2D::single_step, "single run of optimization")
        .def("getValue", &NelderMead2D::get_value, "returns max value")
        .def("getPoint", &NelderMead2D::get_point, "returns max point");

    class_<ConfigItem, ConfigItem*, boost::noncopyable>("ConfigItem", no_init)
        .def_readonly("name", &ConfigItem::name);

    class_<Configuration, std::shared_ptr<Configuration>, boost::noncopyable>("Configuration")
        .def("FromRegisteredConfigurables", &Configuration::from_registered_configurables)
        .def("nameLookup", &Configuration::name_lookup,
             return_value_policy<reference_existing_object>())
        .staticmethod("FromRegisteredConfigurables");

    // Add wrappers for ConfigItem subclasses
    class_<ConfigBool, ConfigBool*, boost::noncopyable, bases<ConfigItem>>("ConfigBool", no_init)
        .add_property("value", &ConfigBool::value, &ConfigBool::set_value)
        .def("__str__", &ConfigBool::to_string);

    class_<ConfigDouble, ConfigDouble*, boost::noncopyable, bases<ConfigItem>>("ConfigDouble",
                                                                               no_init)
        .add_property("value", &ConfigDouble::value, &ConfigDouble::set_value)
        .def("__str__", &ConfigDouble::to_string);

    class_<ConfigInt, ConfigInt*, boost::noncopyable, bases<ConfigItem>>("ConfigInt", no_init)
        .add_property("value", &ConfigInt::value, &ConfigInt::set_value)
        .def("__str__", &ConfigInt::to_string);

    class_<MotionConstraints>("MotionConstraints")
        .add_property("MaxRobotSpeed", &MotionConstraints::max_speed)
        .add_property("MaxRobotAccel", &MotionConstraints::max_acceleration);

    enum_<RefereeModuleEnums::Command>("Command")
        .value("halt", SSL_Referee_Command_HALT)
        .value("stop", SSL_Referee_Command_STOP)
        .value("normal_start", SSL_Referee_Command_NORMAL_START)
        .value("force_start", SSL_Referee_Command_FORCE_START)
        .value("prepare_kickoff_yellow", SSL_Referee_Command_PREPARE_KICKOFF_YELLOW)
        .value("prepare_kickoff_blue", SSL_Referee_Command_PREPARE_KICKOFF_BLUE)
        .value("prepare_penalty_yellow", SSL_Referee_Command_PREPARE_PENALTY_YELLOW)
        .value("prepare_penalty_blue", SSL_Referee_Command_PREPARE_PENALTY_BLUE)
        .value("direct_free_yellow", SSL_Referee_Command_DIRECT_FREE_YELLOW)
        .value("direct_free_blue", SSL_Referee_Command_DIRECT_FREE_BLUE)
        .value("indirect_free_yellow", SSL_Referee_Command_INDIRECT_FREE_YELLOW)
        .value("indirect_free_blue", SSL_Referee_Command_INDIRECT_FREE_BLUE)
        .value("timeout_yellow", SSL_Referee_Command_TIMEOUT_YELLOW)
        .value("timeout_blue", SSL_Referee_Command_TIMEOUT_BLUE)
        .value("goal_yellow", SSL_Referee_Command_GOAL_YELLOW)
        .value("goal_blue", SSL_Referee_Command_GOAL_BLUE)
        .value("ball_placement_yellow", SSL_Referee_Command_BALL_PLACEMENT_YELLOW)
        .value("ball_placement_blue", SSL_Referee_Command_BALL_PLACEMENT_BLUE);
}
