#include <cmath>
#include <iostream>

#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, world_robot, robot_merger_power,
                  1.5,
                  "Multiplier to scale the weighted average coefficient "
                  "to be nonlinear.")
using world_robot::PARAM_robot_merger_power;

WorldRobot::WorldRobot() : isValid(false) {}

WorldRobot::WorldRobot(RJ::Time calc_time, Team team, int robot_id,
                       const std::list<KalmanRobot>& kalman_robots)
    : team(team), robotID(robot_id), isValid(true), time(calc_time) {
    // Theta's are converted to rect coords then back to polar to convert
    Geometry2d::Point pos_cartesian_avg;
    Geometry2d::Point theta_cartesian_avg;
    Geometry2d::Twist twist_avg;

    double total_pos_weight = 0;
    double total_vel_weight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (PARAM_robot_merger_power < 1 || PARAM_robot_merger_power > 2) {
        std::cout << "WARN: robot_merger_power must be between 1 and 2"
                  << std::endl;
    }

    if (kalman_robots.empty()) {
        throw std::runtime_error(
            "ERROR: Zero robots are given to the WorldRobot constructor");
    }

    for (const KalmanRobot& robot : kalman_robots) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        Geometry2d::Pose pose_cov{robot.getPosCov(), robot.getThetaCov()};
        Geometry2d::Twist twist_cov{robot.getVelCov(), robot.getOmegaCov()};

        // Std dev of each state
        // Lower std dev gives better idea of true values
        Geometry2d::Pose pose_std_dev;
        Geometry2d::Twist twist_std_dev;
        pose_std_dev.position().x() = std::sqrt(pose_cov.position().x());
        pose_std_dev.position().y() = std::sqrt(pose_cov.position().y());
        twist_std_dev.linear().x() = std::sqrt(twist_cov.linear().x());
        twist_std_dev.linear().y() = std::sqrt(twist_cov.linear().y());
        pose_std_dev.heading() = std::sqrt(pose_cov.heading());
        twist_std_dev.angular() = std::sqrt(pose_cov.heading());

        // Inversely proportional to how much the filter has been updated
        double filter_uncertantity = 1.0 / robot.getHealth();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double pos_uncertantity =
            std::sqrt(pose_std_dev.position().magsq() +
                      std::pow(pose_std_dev.heading(), 2));
        double vel_uncertantity =
            std::sqrt(pose_std_dev.position().magsq() +
                      std::pow(twist_std_dev.angular(), 2));

        double filter_pos_weight = std::pow(
            pos_uncertantity * filter_uncertantity, -PARAM_robot_merger_power);

        double filter_vel_weight = std::pow(
            vel_uncertantity * filter_uncertantity, -PARAM_robot_merger_power);

        pos_cartesian_avg += filter_pos_weight * robot.getPos();
        theta_cartesian_avg +=
            Geometry2d::Point(filter_pos_weight * cos(robot.getTheta()),
                              filter_pos_weight * sin(robot.getTheta()));
        twist_avg.linear() += filter_vel_weight * robot.getVel();
        twist_avg.angular() += filter_vel_weight * robot.getOmega();

        total_pos_weight += filter_pos_weight;
        total_vel_weight += filter_vel_weight;
    }

    pos_cartesian_avg /= total_pos_weight;
    theta_cartesian_avg /= total_pos_weight;
    twist_avg.linear() /= total_vel_weight;
    twist_avg.angular() /= total_vel_weight;

    pose.position() = pos_cartesian_avg;
    pose.heading() = atan2(theta_cartesian_avg.y(), theta_cartesian_avg.x());
    twist.linear() = twist_avg.linear();
    twist.angular() = twist_avg.angular();
    posCov = total_pos_weight / kalman_robots.size();
    velCov = total_vel_weight / kalman_robots.size();
    robotComponents = kalman_robots;
}

bool WorldRobot::getIsValid() const { return isValid; }

int WorldRobot::getRobotID() const { return robotID; }

Geometry2d::Point WorldRobot::getPos() const { return pose.position(); }

double WorldRobot::getTheta() const { return pose.heading(); }

Geometry2d::Pose WorldRobot::getPose() const { return pose; }

Geometry2d::Point WorldRobot::getVel() const { return twist.linear(); }

double WorldRobot::getOmega() const { return twist.angular(); }

Geometry2d::Twist WorldRobot::getTwist() const { return twist; }

double WorldRobot::getPosCov() const { return posCov; }

double WorldRobot::getVelCov() const { return velCov; }

const std::list<KalmanRobot>& WorldRobot::getRobotComponents() const {
    return robotComponents;
}

RJ::Time WorldRobot::getTime() const { return time; }
}  // namespace vision_filter