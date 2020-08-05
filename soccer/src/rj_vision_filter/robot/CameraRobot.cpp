#include <iostream>

#include <rj_common/Utils.hpp>
#include <rj_vision_filter/robot/CameraRobot.hpp>

namespace vision_filter {
CameraRobot::CameraRobot(const RJ::Time& time_captured,
                         const DetectionRobotMsg& msg,
                         const Geometry2d::TransformMatrix& world_to_team,
                         double team_angle)
    : timeCaptured{time_captured},
      pose{world_to_team * Geometry2d::Point{msg.x / 1000, msg.y / 1000},
           fixAngleRadians(msg.orientation + team_angle)},
      robotID{static_cast<int>(msg.robot_id)} {}

RJ::Time CameraRobot::getTimeCaptured() const { return timeCaptured; }

Geometry2d::Point CameraRobot::getPos() const { return pose.position(); }

double CameraRobot::getTheta() const { return pose.heading(); }

int CameraRobot::getRobotID() const { return robotID; }

Geometry2d::Pose CameraRobot::getPose() const { return pose; }

CameraRobot CameraRobot::CombineRobots(const std::list<CameraRobot>& robots) {
    // Make sure we don't divide by zero due to some weird error
    if (robots.empty()) {
        std::cout << "ERROR: Number of robots to combine is zero" << std::endl;

        return CameraRobot(RJ::now(), Geometry2d::Pose(), -1);
    }

    // Have to do the average like Ti + sum(Tn - Ti)/N
    // so that we aren't trying to add time_points. It's durations instead.
    RJ::Time init_time = robots.front().getTimeCaptured();
    RJ::Seconds time_avg = RJ::Seconds(0);
    // Adding angles are done through conversion to rect coords then back to
    // polar
    Geometry2d::Point pos_avg;
    Geometry2d::Point theta_cartesian_avg;
    int robot_id = -1;

    for (const CameraRobot& cr : robots) {
        time_avg += RJ::Seconds(cr.getTimeCaptured() - init_time);
        pos_avg += cr.getPos();
        theta_cartesian_avg +=
            Geometry2d::Point(cos(cr.getTheta()), sin(cr.getTheta()));
        robot_id =
            cr.getRobotID();  // Shouldn't change besides the first iteration
    }

    time_avg /= robots.size();
    pos_avg /= robots.size();
    theta_cartesian_avg /= robots.size();

    return CameraRobot(
        init_time + time_avg,
        {pos_avg, atan2(theta_cartesian_avg.y(), theta_cartesian_avg.x())},
        robot_id);
}
}  // namespace vision_filter