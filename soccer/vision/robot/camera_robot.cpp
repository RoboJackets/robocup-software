
#include <spdlog/spdlog.h>

#include <rj_common/utils.hpp>

#include "vision/robot/camera_robot.hpp"

namespace vision_filter {
CameraRobot::CameraRobot(const RJ::Time& time_captured, const DetectionRobotMsg& msg,
                         const rj_geometry::TransformMatrix& world_to_team, double team_angle)
    : time_captured_{time_captured},
      pose_{world_to_team * rj_geometry::Point{msg.x / 1000, msg.y / 1000},
            fix_angle_radians(msg.orientation + team_angle)},
      robot_id_{static_cast<int>(msg.robot_id)} {}

RJ::Time CameraRobot::get_time_captured() const { return time_captured_; }

rj_geometry::Point CameraRobot::get_pos() const { return pose_.position(); }

double CameraRobot::get_theta() const { return pose_.heading(); }

int CameraRobot::get_robot_id() const { return robot_id_; }

rj_geometry::Pose CameraRobot::get_pose() const { return pose_; }

CameraRobot CameraRobot::combine_robots(const std::list<CameraRobot>& robots) {
    // Make sure we don't divide by zero due to some weird error
    if (robots.empty()) {
        SPDLOG_ERROR("Number of robots to combine is zero");

        return CameraRobot(RJ::now(), rj_geometry::Pose(), -1);
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time init_time = robots.front().get_time_captured();
    RJ::Seconds time_avg = RJ::Seconds(0);
    // Adding angles are done through conversion to rect coords then back to
    // polar
    rj_geometry::Point pos_avg;
    rj_geometry::Point theta_cartesian_avg;
    int robot_id = -1;

    for (const CameraRobot& cr : robots) {
        time_avg += RJ::Seconds(cr.get_time_captured() - init_time);
        pos_avg += cr.get_pos();
        theta_cartesian_avg += rj_geometry::Point(cos(cr.get_theta()), sin(cr.get_theta()));
        robot_id = cr.get_robot_id();  // Shouldn't change besides the first iteration
    }

    time_avg /= robots.size();
    pos_avg /= robots.size();
    theta_cartesian_avg /= robots.size();

    return CameraRobot(init_time + time_avg,
                       {pos_avg, atan2(theta_cartesian_avg.y(), theta_cartesian_avg.x())},
                       robot_id);
}
}  // namespace vision_filter