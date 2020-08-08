#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <list>
#include <rj_vision_filter/robot/kalman_robot.hpp>

namespace vision_filter {
class KalmanRobot;

class WorldRobot {
public:
    enum Team { YELLOW, BLUE };

    /**
     * Creates an invalid world robot.
     * This is so the World can create a full list of robots without dealing
     * with holes It's a little less efficient, but it makes things much cleaner
     * code-wise
     */
    WorldRobot();

    /**
     * Creates a valid world robot
     *
     * @param robot_id The ID of the robot
     * @param team The team color
     * @param kalman_robots List of kalman robots from each of the cameras to
     * merger
     */
    WorldRobot(RJ::Time calc_time, Team team, int robot_id,
               const std::list<KalmanRobot>& kalman_robots);

    /**
     * @return If the robot actually represents a real robot
     */
    bool get_is_valid() const;

    /**
     * @return Enum value representing team color
     */
    Team get_team_color() const;

    /**
     * @return The robot id
     */
    int get_robot_id() const;

    /**
     * @return The best estimated position of the robot
     */
    rj_geometry::Point get_pos() const;

    /**
     * @return The best estimated heading of the robot
     */
    double get_theta() const;

    /**
     * @return The best estimated pose of the robot
     */
    rj_geometry::Pose get_pose() const;

    /**
     * @return The best estimated velocity of the robot
     */
    rj_geometry::Point get_vel() const;

    /**
     * @return The best estimated angular velocity of the robot
     */
    double get_omega() const;

    /**
     * @return The best estimated twist of the robot
     */
    rj_geometry::Twist get_twist() const;

    /**
     * @return The average position covariance of the filter including theta
     */
    double get_pos_cov() const;

    /**
     * @return The average velocity covariance of the filter including omega
     */
    double get_vel_cov() const;

    /**
     * @return List of all the building kalman robots for this world robot
     */
    const std::list<KalmanRobot>& get_robot_components() const;

    /**
     * @return Time of creation for the robot estimate
     */
    RJ::Time get_time() const;

private:
    Team team_;
    int robot_id_{};
    rj_geometry::Pose pose_;
    rj_geometry::Twist twist_;
    double pos_cov_{};
    double vel_cov_{};
    std::list<KalmanRobot> robot_components_;
    RJ::Time time_;

    bool is_valid_;
};
}  // namespace vision_filter