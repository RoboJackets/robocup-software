#include <iostream>

#include <rj_common/Utils.hpp>
#include <rj_vision_filter/robot/CameraRobot.hpp>

namespace vision_filter {
CameraRobot::CameraRobot(const RJ::Time& time_captured, const DetectionRobotMsg& msg,
                         const Geometry2d::TransformMatrix& world_to_team, double team_angle)
    : time_captured_{time_captured},
      pose_{world_to_team * Geometry2d::Point{msg.x / 1000, msg.y / 1000},
           fix_angle_radians(msg.orientation + team_angle)},
      robot_id_{static_cast<int>(msg.robot_id)} {}

RJ::Time CameraRobot::get_time_captured() const { return time_captured_; }

Geometry2d::Point CameraRobot::get_pos() const { return pose_.position(); }

double CameraRobot::get_theta() const { return pose_.heading(); }

int CameraRobot::get_robot_id() const { return robot_id_; }

Geometry2d::Pose CameraRobot::get_pose() const { return pose_; }

CameraRobot CameraRobot::combine_robots(const std::list<CameraRobot>& robots) {
    // Make sure we don't divide by zero due to some weird error
    if (robots.empty()) {
        std::cout << "ERROR: Number of robots to combine is zero" << std::endl;

        return CameraRobot(RJ::now(), Geometry2d::Pose(), -1);
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time init_time = robots.front().get_time_captured();
    RJ::Seconds time_avg = RJ::Seconds(0);
    // Adding angles are done through conversion to rect coords then back to
    // polar
    Geometry2d::Point pos_avg;
    Geometry2d::Point theta_cartesian_avg;
    int robot_id_ = -1;

    for (const CameraRobot& cr : robots) {
        time_avg += RJ::Seconds(cr.get_time_captured() - init_time);
        pos_avg += cr.get_pos();
        theta_cartesian_avg += Geometry2d::Point(cos(cr.get_theta()), sin(cr.get_theta()));
        robot_id_ = cr.get_robot_id();  // Shouldn't change besides the first iteration
    }

    time_avg /= robots.size();
    pos_avg /= robots.size();
    theta_cartesian_avg /= robots.size();

    return CameraRobot(init_time + time_avg,
                       {pos_avg, atan2(theta_cartesian_avg.y(), theta_cartesian_avg.x())},
                       robot_id_);
}
}  // namespace vision_filter