#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, world_robot, robot_merger_power, 1.5,
                  "Multiplier to scale the weighted average coefficient "
                  "to be nonlinear.")
using world_robot::PARAM_robot_merger_power;

WorldRobot::WorldRobot() : is_valid_(false) {}

WorldRobot::WorldRobot(RJ::Time calc_time, Team team, int robot_id,
                       const std::list<KalmanRobot>& kalman_robots)
    : team_(team), robot_id_(robot_id), time_(calc_time), is_valid_(true) {
    // Theta's are converted to rect coords then back to polar to convert
    rj_geometry::Point pos_cartesian_avg;
    rj_geometry::Point theta_cartesian_avg;
    rj_geometry::Twist twist_avg;

    double total_pos_weight = 0;
    double total_vel_weight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (PARAM_robot_merger_power < 1 || PARAM_robot_merger_power > 2) {
        SPDLOG_WARN("robot_merger_power must be between 1 and 2");
    }

    if (kalman_robots.empty()) {
        throw std::runtime_error("ERROR: Zero robots are given to the WorldRobot constructor");
    }

    for (const KalmanRobot& robot : kalman_robots) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        rj_geometry::Pose pose_cov{robot.get_pos_cov(), robot.get_theta_cov()};
        rj_geometry::Twist twist_cov{robot.get_vel_cov(), robot.get_omega_cov()};

        // Std dev of each state
        // Lower std dev gives better idea of true values
        rj_geometry::Pose pose_std_dev;
        rj_geometry::Twist twist_std_dev;
        pose_std_dev.position().x() = std::sqrt(pose_cov.position().x());
        pose_std_dev.position().y() = std::sqrt(pose_cov.position().y());
        twist_std_dev.linear().x() = std::sqrt(twist_cov.linear().x());
        twist_std_dev.linear().y() = std::sqrt(twist_cov.linear().y());
        pose_std_dev.heading() = std::sqrt(pose_cov.heading());
        twist_std_dev.angular() = std::sqrt(pose_cov.heading());

        // Inversely proportional to how much the filter has been updated
        double filter_uncertantity = 1.0 / robot.get_health();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double pos_uncertantity =
            std::sqrt(pose_std_dev.position().magsq() + std::pow(pose_std_dev.heading(), 2));
        double vel_uncertantity =
            std::sqrt(pose_std_dev.position().magsq() + std::pow(twist_std_dev.angular(), 2));

        double filter_pos_weight =
            std::pow(pos_uncertantity * filter_uncertantity, -PARAM_robot_merger_power);

        double filter_vel_weight =
            std::pow(vel_uncertantity * filter_uncertantity, -PARAM_robot_merger_power);

        pos_cartesian_avg += filter_pos_weight * robot.get_pos();
        theta_cartesian_avg += rj_geometry::Point(filter_pos_weight * cos(robot.get_theta()),
                                                  filter_pos_weight * sin(robot.get_theta()));
        twist_avg.linear() += filter_vel_weight * robot.get_vel();
        twist_avg.angular() += filter_vel_weight * robot.get_omega();

        total_pos_weight += filter_pos_weight;
        total_vel_weight += filter_vel_weight;
    }

    pos_cartesian_avg /= total_pos_weight;
    theta_cartesian_avg /= total_pos_weight;
    twist_avg.linear() /= total_vel_weight;
    twist_avg.angular() /= total_vel_weight;

    pose_.position() = pos_cartesian_avg;
    pose_.heading() = atan2(theta_cartesian_avg.y(), theta_cartesian_avg.x());
    twist_.linear() = twist_avg.linear();
    twist_.angular() = twist_avg.angular();
    pos_cov_ = total_pos_weight / kalman_robots.size();
    vel_cov_ = total_vel_weight / kalman_robots.size();
    robot_components_ = kalman_robots;
}

bool WorldRobot::get_is_valid() const { return is_valid_; }

int WorldRobot::get_robot_id() const { return robot_id_; }

rj_geometry::Point WorldRobot::get_pos() const { return pose_.position(); }

double WorldRobot::get_theta() const { return pose_.heading(); }

rj_geometry::Pose WorldRobot::get_pose() const { return pose_; }

rj_geometry::Point WorldRobot::get_vel() const { return twist_.linear(); }

double WorldRobot::get_omega() const { return twist_.angular(); }

rj_geometry::Twist WorldRobot::get_twist() const { return twist_; }

double WorldRobot::get_pos_cov() const { return pos_cov_; }

double WorldRobot::get_vel_cov() const { return vel_cov_; }

const std::list<KalmanRobot>& WorldRobot::get_robot_components() const { return robot_components_; }

RJ::Time WorldRobot::get_time() const { return time_; }
}  // namespace vision_filter